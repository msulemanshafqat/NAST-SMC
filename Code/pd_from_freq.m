function [Kp,Kd,wn] = pd_from_freq(m_or_J,f_target,zeta)
    wn = 2*pi*f_target;
    Kp = m_or_J * wn^2;
    Kd = 2*zeta*sqrt(m_or_J*Kp);
end