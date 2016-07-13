function A_fwd_y = A_fwd_yawed_right(r,d,mm,mt,It,Im,phi,theta,alpha1,beta1,beta2,l1)
%A_FWD_YAWED_RIGHT
%    A_FWD_Y = A_FWD_YAWED_RIGHT(R,D,G,MM,MT,IT,IM,KP_G,KD_G,X,Y,Z,PHI,THETA,PSI,ALPHA1,BETA1,BETA2,L1)

%    This function was generated by the Symbolic Math Toolbox version 5.6.
%    27-Apr-2015 18:17:56

t2 = beta1+phi;
t3 = beta2-phi;
t4 = sin(t2);
t5 = sin(phi);
t6 = d.*t5.*2.0;
t7 = sin(t3);
t8 = r.*t7;
t21 = r.*t4;
t9 = -t21+t6+t8;
t10 = sin(theta);
t11 = mm.*2.0;
t12 = mt+t11;
t13 = cos(t2);
t14 = r.*t13;
t15 = cos(phi);
t16 = cos(t3);
t17 = r.*t16;
t19 = d.*t15.*2.0;
t18 = t14+t17-t19;
t20 = cos(theta);
t22 = mm.*t10.*t18;
t23 = mm.*t10.*t9;
t24 = mm.*t20.*t9;
t25 = Im.*2.0;
t26 = d.^2;
t27 = mm.*t26.*2.0;
t28 = r.^2;
t29 = mm.*t28.*2.0;
t30 = cos(beta1);
t31 = cos(beta2);
t44 = d.*mm.*r.*t30.*2.0;
t45 = d.*mm.*r.*t31.*2.0;
t32 = It+t25+t27+t29-t44-t45;
t33 = mm.*t28;
t34 = sin(beta1);
t37 = r.*t30;
t35 = d-t37;
t36 = cos(alpha1);
t38 = mm.*t18.*t20;
t39 = phi.*2.0;
t40 = d.*mm.*r.*t31;
t41 = sin(beta2);
t42 = t15.^2;
t43 = sin(alpha1);
t46 = d.*r.*t34;
t47 = t28.*t31.*t41;
t48 = t15.*t26.*t5.*2.0;
t49 = t30.^2;
t50 = t15.*t28.*t49.*t5.*2.0;
t51 = t31.^2;
t52 = t15.*t28.*t5.*t51.*2.0;
t53 = d.*r.*t41.*t42.*2.0;
t54 = t28.*t30.*t34.*t42.*2.0;
t55 = t46+t47+t48+t50+t52+t53+t54-d.*r.*t41-t15.*t28.*t5.*2.0-t28.*t30.*t34-d.*r.*t34.*t42.*2.0-t28.*t31.*t41.*t42.*2.0-d.*r.*t15.*t30.*t5.*2.0-d.*r.*t15.*t31.*t5.*2.0;
t56 = mm.*t20.*t55;
t57 = t20.^2;
t69 = d.*mm.*r.*t30;
t58 = Im+t33-t69;
t59 = t20.*t43;
t60 = t10.*t30.*t5;
t61 = t10.*t15.*t34;
t62 = t60+t61;
t63 = t36.*t62;
t64 = t59+t63;
t65 = l1.*t64;
t66 = t20.*t30.*t5;
t67 = t15.*t20.*t34;
t68 = t66+t67;
t70 = alpha1-beta1-phi;
t71 = cos(t70);
t72 = l1.*t71;
t73 = alpha1+beta1+phi;
t74 = cos(t73);
t75 = l1.*t74;
t95 = r.*t4.*2.0;
t76 = t72+t75-t95;
t77 = mm.*r.*t16;
t78 = -Im-t33+t40;
t79 = Im+t33-t40;
t80 = t10.*t79;
t81 = Im+t33;
t82 = t10.*t43;
t86 = t36.*t68;
t83 = t82-t86;
t84 = t10.*t15.*t30;
t85 = r.*t10.*t15.*t34;
t87 = l1.*t83;
t88 = t15.*t20.*t35;
t89 = r.*t20.*t34.*t5;
t90 = t35.*t5;
t91 = t15.*t30;
t92 = t91-t34.*t5;
t93 = l1.*t36.*t92;
t94 = l1.*t43.*t62;
t96 = t30.*t5;
t97 = t15.*t34;
t98 = t96+t97;
t99 = l1.*t36.*t98;
t100 = t10.*t15.*t35;
t101 = r.*t10.*t34.*t5;
t102 = l1.*t13.*t43;
t103 = l1.*t36.*t4;
t104 = t15.*t20.*t30;
t105 = t104-t20.*t34.*t5;
t106 = r.*t15.*t20.*t34;
t107 = t100+t101-t65;
t108 = l1.*t10.*t36;
t109 = l1.*t43.*t68;
A_fwd_y = reshape([t12,0.0,0.0,t23,t38,-mm.*t9,0.0,-mm.*r.*t10.*t4,-mm.*r.*t10.*t7,0.0,1.0,0.0,0.0,0.0,t12,0.0,-mm.*t18,0.0,t22,0.0,-mm.*r.*t13,t77,0.0,0.0,1.0,0.0,0.0,0.0,t12,t24,-t22,0.0,0.0,-mm.*r.*t20.*t4,-mm.*r.*t20.*t7,0.0,0.0,0.0,1.0,t23,-mm.*t18,t24,t32,0.0,-t10.*t32,0.0,t58,t78,0.0,-t85+t10.*t35.*t5+l1.*t36.*(t84-t10.*t34.*t5),-t99+t15.*t35+r.*t34.*t5,-t106+l1.*t105.*t36+t20.*t35.*t5,t38,0.0,-t22,0.0,It+t25+t33-t40+mm.*t26+mm.*t26.*cos(t39)+mm.*t28.*cos(beta1.*2.0+t39).*(1.0./2.0)+mm.*t28.*cos(beta2.*2.0-t39).*(1.0./2.0)-d.*mm.*r.*t30-d.*mm.*r.*cos(beta1+t39)-d.*mm.*r.*cos(beta2-t39),t56,0.0,0.0,0.0,0.0,-t87-t88-t89,0.0,t107,-mm.*t9,t22,0.0,-t10.*t32,t56,It+t25+t27+t29-t44-t45-mm.*t28.*t57.*2.0-mm.*t26.*t42.*t57.*2.0+mm.*t28.*t42.*t57.*2.0+mm.*t28.*t49.*t57+mm.*t28.*t51.*t57-mm.*t28.*t42.*t49.*t57.*2.0-mm.*t28.*t42.*t51.*t57.*2.0+d.*mm.*r.*t30.*t42.*t57.*2.0+d.*mm.*r.*t31.*t42.*t57.*2.0-d.*mm.*r.*t15.*t34.*t5.*t57.*2.0+d.*mm.*r.*t15.*t41.*t5.*t57.*2.0+mm.*t15.*t28.*t30.*t34.*t5.*t57.*2.0-mm.*t15.*t28.*t31.*t41.*t5.*t57.*2.0,0.0,-t10.*t58,t80,0.0,-t90-t93+r.*t15.*t34,-t100-t101+t65,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t94+l1.*t20.*t36,-t102,-t108-t109,-mm.*r.*t10.*t4,-mm.*r.*t13,-mm.*r.*t20.*t4,t58,0.0,-t10.*t58,0.0,t81,0.0,0.0,t10.*t76.*(1.0./2.0),-t103-t14,t20.*t76.*(1.0./2.0),-mm.*r.*t10.*t7,t77,-mm.*r.*t20.*t7,t78,0.0,t80,0.0,0.0,t81,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t64,t13.*t36,-t82+t86,-1.0,0.0,0.0,t85-t10.*t35.*t5-l1.*t36.*(t84-t10.*t34.*t5),t87+t88+t89,t90+t93-r.*t15.*t34,t94-l1.*t20.*t36,t10.*t76.*(-1.0./2.0),0.0,-t59-t63,0.0,0.0,0.0,0.0,-1.0,0.0,t99-t15.*t35-r.*t34.*t5,0.0,t107,t102,t103+t14,0.0,-t13.*t36,0.0,0.0,0.0,0.0,0.0,-1.0,t106-l1.*t105.*t36-t20.*t35.*t5,t65-t10.*t15.*t35-r.*t10.*t34.*t5,0.0,t108+t109,t20.*t76.*(-1.0./2.0),0.0,t83,0.0,0.0,0.0],[13, 13]);
