from __future__ import annotations

import os
import sys
import struct
import threading
import pandas as pd
import tkinter as tk
from tqdm import tqdm
import customtkinter as ctk
from dataclasses import dataclass
from matplotlib.figure import Figure
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import Dict, List, Optional, Tuple
from tkinter import filedialog, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends._backend_tk import NavigationToolbar2Tk
import numpy as np

START_BYTE = 0xFE
END_BYTE = 0xFF
PKT_LEN = 64
MSGID_JOG = 0x0B
MSGID_STATUS = 0xFF


@dataclass(frozen=True)
class PacketLayout:
    axis1_offset: int = 10
    axis2_offset: int = 33


def crc16_xmodem(data: bytes, init: int = 0x0000) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_axis(buf: bytes, offset: int) -> Dict[str, float | int]:
    mode = buf[offset + 0]
    flags = buf[offset + 1]
    setpoint, theta, omega, tau = struct.unpack_from("<ffff", buf, offset + 2)
    temp = struct.unpack_from("<b", buf, offset + 18)[0]
    rtt = struct.unpack_from("<H", buf, offset + 19)[0]
    tx_err = buf[offset + 21]
    timeouts = buf[offset + 22]

    return {
        "mode": mode,
        "flags": flags,
        "setpoint": setpoint,
        "theta": theta,
        "omega": omega,
        "tau": tau,
        "temp": temp,
        "rtt": rtt,
        "txErr": tx_err,
        "timeouts": timeouts,
    }


def extract_axis_dfs(bin_path: str, verify_crc: bool = True) -> Tuple[Dict[int, pd.DataFrame], pd.DataFrame]:
    with open(bin_path, "rb") as file:
        data = file.read()

    rows_by_axis: Dict[int, List[dict]] = {}
    jog_rows: List[dict] = []
    layout = PacketLayout()

    total_bytes = len(data)
    scan_limit = total_bytes - PKT_LEN + 1
    skip_until = 0

    for index in tqdm(range(max(scan_limit, 0)), desc=f"Parsing {os.path.basename(bin_path)}", unit="byte"):
        if index < skip_until:
            continue
        if data[index] != START_BYTE:
            continue
        if data[index + 63] != END_BYTE:
            continue

        packet = data[index : index + PKT_LEN]

        if verify_crc:
            expected_crc = struct.unpack_from("<H", packet, 61)[0]
            actual_crc = crc16_xmodem(packet[0:61])
            if actual_crc != expected_crc:
                continue

        from_id = packet[1]
        to_id = packet[2]
        seq = packet[3]
        msgid = packet[4]
        if msgid == MSGID_STATUS:
            timestamp_us = struct.unpack_from("<I", packet, 5)[0]
            resflag = packet[9]

            axis1_no = 2 * from_id - 1
            axis2_no = 2 * from_id

            common = {
                "from_id": from_id,
                "to_id": to_id,
                "seq": seq,
                "msgid": msgid,
                "timestamp_us": timestamp_us,
                "resflag": resflag,
            }

            axis1 = parse_axis(packet, layout.axis1_offset)
            axis2 = parse_axis(packet, layout.axis2_offset)

            axis1_row = common.copy()
            axis1_row["axis_no"] = axis1_no
            axis1_row.update(axis1)
            rows_by_axis.setdefault(axis1_no, []).append(axis1_row)

            axis2_row = common.copy()
            axis2_row["axis_no"] = axis2_no
            axis2_row.update(axis2)
            rows_by_axis.setdefault(axis2_no, []).append(axis2_row)
        elif msgid == MSGID_JOG:
            j1, j2, j3, j4, j5, j6 = struct.unpack_from("<6f", packet, 5)
            jog_rows.append(
                {
                    "packet_index": index,
                    "from_id": from_id,
                    "to_id": to_id,
                    "seq": seq,
                    "msgid": msgid,
                    "j1": j1,
                    "j2": j2,
                    "j3": j3,
                    "j4": j4,
                    "j5": j5,
                    "j6": j6,
                }
            )

        skip_until = index + PKT_LEN

    dfs = {axis: pd.DataFrame(rows) for axis, rows in rows_by_axis.items()}

    for axis, df in dfs.items():
        if not df.empty:
            dfs[axis] = df.sort_values(["timestamp_us", "seq"], kind="stable").reset_index(drop=True)

    jog_df = pd.DataFrame(jog_rows)
    if not jog_df.empty:
        jog_df = jog_df.sort_values(["packet_index", "seq"], kind="stable").reset_index(drop=True)

    return dfs, jog_df


# Module-level function — must be picklable for ProcessPoolExecutor (Windows spawn).
def _export_file_task(args: Tuple) -> Tuple[str, Optional[str]]:
    out_path, dfs, jog_df = args
    try:
        with pd.ExcelWriter(out_path) as writer:
            for i in range(1, 7):
                df = dfs.get(i)
                (pd.DataFrame() if df is None else df).to_excel(writer, sheet_name=f"Axis_{i}", index=False)
            jog_df.to_excel(writer, sheet_name="JOG", index=False)
        return out_path, None
    except Exception as exc:
        return out_path, str(exc)


def prompt_for_bin_files() -> List[str]:
    dialog_root = tk.Tk()
    dialog_root.withdraw()

    paths = filedialog.askopenfilenames(
        initialdir=os.path.join(os.path.dirname(__file__), "output"),
        title="Select log file(s)",
        filetypes=[("Binary files", "*.bin"), ("All files", "*.*")],
        parent=dialog_root,
    )

    dialog_root.destroy()
    return list(paths)


FileData = Tuple[str, Dict[int, pd.DataFrame], pd.DataFrame]


class LogDecoderApp:
    def __init__(self, file_data: List[FileData]) -> None:
        self.file_data = file_data
        self.app_closing = False
        self._axis_tabviews: Dict[str, ctk.CTkTabview] = {}

        ctk.set_appearance_mode("system")
        ctk.set_default_color_theme("blue")

        self.root = ctk.CTk()
        self.root.title("Axis Plots")
        self.root.geometry("1280x720")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.file_tabview = ctk.CTkTabview(self.root)
        self.file_tabview.pack(fill="both", expand=True, padx=8, pady=(8, 4))

        controls = ctk.CTkFrame(self.root)
        controls.pack(side="bottom", fill="x", padx=8, pady=(4, 8))

        self.export_current_btn = ctk.CTkButton(controls, text="Export Current", command=self.export_current)
        self.export_current_btn.pack(side="left", padx=8, pady=8)

        self.export_all_btn = ctk.CTkButton(controls, text="Export All", command=self.export_all)
        self.export_all_btn.pack(side="left", padx=4, pady=8)

        self.status_label = ctk.CTkLabel(controls, text="")
        self.status_label.pack(side="left", padx=8, pady=8)

        self._build_file_tabs()
        self.root.after(50, self._prerender_all)

    def on_close(self) -> None:
        self.app_closing = True
        self.root.quit()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()

    def _build_file_tabs(self) -> None:
        for bin_path, dfs, jog_df in self.file_data:
            tab_name = os.path.basename(bin_path)
            self.file_tabview.add(tab_name)
            tab = self.file_tabview.tab(tab_name)
            axis_tv = self._build_axis_tabs_in(tab, dfs, jog_df)
            self._axis_tabviews[tab_name] = axis_tv

    def _build_axis_tabs_in(
        self, parent: ctk.CTkFrame, dfs: Dict[int, pd.DataFrame], jog_df: pd.DataFrame
    ) -> ctk.CTkTabview:
        axis_tabview = ctk.CTkTabview(parent)
        axis_tabview.pack(fill="both", expand=True)

        for axis in range(1, 7):
            df = dfs.get(axis)
            tab_name = f"Axis {axis}"
            axis_tabview.add(tab_name)
            tab = axis_tabview.tab(tab_name)

            if df is None or df.empty:
                msg = ctk.CTkLabel(tab, text=f"No data for axis {axis}")
                msg.pack(padx=12, pady=12)
                continue

            time_s = df["timestamp_us"].to_numpy(dtype="float64") * 1e-6

            fig = Figure()
            axis_plot = fig.add_subplot(111)
            axis_plot.plot(time_s - time_s[0], df["theta"].to_numpy(), label="theta")

            if not jog_df.empty:
                jog_col = f"j{axis}"
                x_jog = np.arange(0, len(jog_df[jog_col])) / 1e3
                axis_plot.plot(x_jog, jog_df[jog_col].to_numpy(dtype="float64"), label="setpoint")

            axis_plot.set_title(f"Axis {axis}: setpoint, theta, & jog command vs time")
            axis_plot.set_xlabel("time (s)")
            axis_plot.set_ylabel("rad")
            axis_plot.grid(True)
            axis_plot.legend(loc="upper right")

            plot_host = tk.Frame(tab)
            plot_host.pack(side="top", fill="both", expand=True)

            toolbar_frame = tk.Frame(plot_host)
            toolbar_frame.pack(side="top", fill="x")

            canvas = FigureCanvasTkAgg(fig, master=plot_host)
            toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
            toolbar.update()
            canvas.draw()
            canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

        return axis_tabview

    def _prerender_all(self) -> None:
        first_file = os.path.basename(self.file_data[0][0]) if self.file_data else None
        for file_basename, axis_tv in self._axis_tabviews.items():
            self.file_tabview.set(file_basename)
            for i in range(1, 7):
                try:
                    axis_tv.set(f"Axis {i}")
                    self.root.update_idletasks()
                except Exception:
                    pass
        if first_file:
            self.file_tabview.set(first_file)
            if first_file in self._axis_tabviews:
                try:
                    self._axis_tabviews[first_file].set("Axis 1")
                except Exception:
                    pass
        self.root.update_idletasks()

    def _get_active_file_data(self) -> Optional[FileData]:
        active_tab = self.file_tabview.get()
        for entry in self.file_data:
            if os.path.basename(entry[0]) == active_tab:
                return entry
        return None

    def _on_export_done(self, out_path: Optional[str] = None, error: Optional[str] = None) -> None:
        if self.app_closing or not self.root.winfo_exists():
            return
        self.export_current_btn.configure(state="normal", text="Export Current")
        self.export_all_btn.configure(state="normal", text="Export All")
        self.status_label.configure(text="")
        if error is None:
            messagebox.showinfo("Export complete", f"Saved:\n{out_path}", parent=self.root)
        else:
            messagebox.showerror("Export failed", f"Could not save:\n{error}", parent=self.root)

    def _set_status(self, text: str) -> None:
        if not self.app_closing and self.root.winfo_exists():
            self.root.after(0, lambda: self.status_label.configure(text=text))

    # Single file: thread + per-sheet tqdm (sequential, no IPC overhead).
    def _export_worker(self, out_path: str, dfs: Dict[int, pd.DataFrame], jog_df: pd.DataFrame) -> None:
        sheets = [(f"Axis_{i}", dfs.get(i)) for i in range(1, 7)] + [("JOG", jog_df)]
        try:
            with pd.ExcelWriter(out_path) as writer:
                for sheet_name, df in tqdm(sheets, desc="Writing sheets"):
                    self._set_status(f"Writing {sheet_name}...")
                    (pd.DataFrame() if df is None else df).to_excel(writer, sheet_name=sheet_name, index=False)
            self.root.after(0, lambda: self._on_export_done(out_path=out_path))
        except Exception as exc:
            self.root.after(0, lambda: self._on_export_done(error=str(exc)))

    # Multi file: ProcessPoolExecutor — N files written in parallel, one process per file.
    def _export_all_worker(self, entries: List[Tuple[str, str, Dict[int, pd.DataFrame], pd.DataFrame]]) -> None:
        n = len(entries)
        args_list = [(out_path, dfs, jog_df) for _, out_path, dfs, jog_df in entries]
        out_to_basename = {out_path: os.path.basename(bin_path) for bin_path, out_path, _, _ in entries}

        saved: List[str] = []
        errors: List[str] = []

        workers = min(n, os.cpu_count() or 4)
        with ProcessPoolExecutor(max_workers=workers) as executor:
            futures = {executor.submit(_export_file_task, args): args[0] for args in args_list}
            for future in tqdm(as_completed(futures), total=n, desc="Exporting files"):
                out_path = futures[future]
                basename = out_to_basename[out_path]
                result_path, err = future.result()
                if err:
                    errors.append(f"{basename}: {err}")
                else:
                    saved.append(result_path)
                done = len(saved) + len(errors)
                self._set_status(f"{done}/{n} done — {basename}")

        def finish() -> None:
            if self.app_closing or not self.root.winfo_exists():
                return
            self.export_current_btn.configure(state="normal", text="Export Current")
            self.export_all_btn.configure(state="normal", text="Export All")
            self.status_label.configure(text="")
            if errors:
                messagebox.showerror("Export errors", "\n".join(errors), parent=self.root)
            else:
                messagebox.showinfo(
                    "Export complete",
                    f"Saved {len(saved)} file(s):\n" + "\n".join(saved),
                    parent=self.root,
                )

        self.root.after(0, finish)

    def export_current(self) -> None:
        entry = self._get_active_file_data()
        if entry is None:
            return
        bin_path, dfs, jog_df = entry
        default_name = f"{os.path.splitext(os.path.basename(bin_path))[0]}_axes.xlsx"
        out_path = filedialog.asksaveasfilename(
            title="Save Excel file",
            defaultextension=".xlsx",
            initialfile=default_name,
            filetypes=[("Excel Workbook", "*.xlsx")],
        )
        if not out_path:
            return
        self.export_current_btn.configure(state="disabled", text="Exporting...")
        self.export_all_btn.configure(state="disabled")
        self.status_label.configure(text="Starting export...")
        threading.Thread(target=self._export_worker, args=(out_path, dfs, jog_df), daemon=True).start()

    def export_all(self) -> None:
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "bin2excel")
        os.makedirs(out_dir, exist_ok=True)
        entries: List[Tuple[str, str, Dict[int, pd.DataFrame], pd.DataFrame]] = []
        for bin_path, dfs, jog_df in self.file_data:
            stem = os.path.splitext(os.path.basename(bin_path))[0]
            entries.append((bin_path, os.path.join(out_dir, f"{stem}_axes.xlsx"), dfs, jog_df))
        self.export_current_btn.configure(state="disabled")
        self.export_all_btn.configure(state="disabled", text="Exporting...")
        self.status_label.configure(text=f"Exporting {len(entries)} file(s) in parallel...")
        threading.Thread(target=self._export_all_worker, args=(entries,), daemon=True).start()


def main() -> int:
    import multiprocessing
    multiprocessing.freeze_support()

    paths = prompt_for_bin_files()
    if not paths:
        print("No files selected. Exiting.")
        return 0

    file_data: List[FileData] = []
    for path in paths:
        dfs, jog_df = extract_axis_dfs(path, verify_crc=False)
        file_data.append((path, dfs, jog_df))

    app = LogDecoderApp(file_data=file_data)
    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
