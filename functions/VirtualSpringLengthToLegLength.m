function [ l_leg, dl_leg, ddl_leg ] = VirtualSpringLengthToLegLength( phi, dphi, ddphi, l_virtual, dl_virtual, ddl_virtual, d_y_com )

    d = d_y_com;
    l_leg = (2.*d.*cos(phi) + sqrt((2.*d.*cos(phi)).^2 + 4.*(l_virtual.^2 - d.^2))) ./ 2;
    dl_leg = (- 2.*dphi.*cos(phi).*sin(phi).*d.^2 + 2.*dl_virtual.*l_virtual)./(2.*(d.^2.*cos(phi).^2 - d.^2 + l_virtual.^2).^(1./2)) - d.*dphi.*sin(phi);
    ddl_leg = (2.^(1./2).*(2.*dl_virtual.^2 + 2.*ddl_virtual.*l_virtual - d.^2.*ddphi.*sin(2.*phi) - 2.*d.^2.*dphi.^2.*cos(2.*phi)))./(2.*(2.*l_virtual.^2 - d.^2 + d.^2.*cos(2.*phi)).^(1./2)) - d.*sin(phi).*ddphi - d.*dphi.^2.*cos(phi) - (2.^(1./2).*(- dphi.*sin(2.*phi).*d.^2 + 2.*dl_virtual.*l_virtual).^2)./(2.*(2.*l_virtual.^2 - d.^2 + d.^2.*cos(2.*phi)).^(3./2));
 
end

