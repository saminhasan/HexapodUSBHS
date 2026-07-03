function filteredSignal = LPFilter(signal, fs, fc)
    w1 = 2 * pi * fc;
    zeta1 = 0.707;
    Bw = [w1^3];
    Aw = conv([1, w1], [1, 2*zeta1*w1, w1^2]);
    Gz = c2d(tf(Bw, Aw), 1/fs);
    Bz = Gz.Numerator{1};
    Az = Gz.Denominator{1};
    filteredSignal = filter(Bz, Az, signal);
end