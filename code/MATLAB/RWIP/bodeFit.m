function rss=bodeFit(x,w,M)
    s=tf('s');
    K=x(3);
    z=x(1);
    wn=x(2);
    sys=K*wn^2/(s^2+2*z*wn*s+wn^2);
    [Mag,~]=bode(sys,w); Mag=Mag(:)'; Mag=20*log10(Mag); 
    rss=sum((Mag-M).^2);
end