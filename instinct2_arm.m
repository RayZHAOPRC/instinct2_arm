%{
Title:inverse kinematic for Instinct2 arm
Author:ZZL
South China University of Technology,2025
Email:13909rayzhao.qq.com
Hi,welcome to contact me and share this code to others if you find it
helpful! DO NOT remove the author's information when using it according to MIT license!
%}
%% 改进型DH参数
% L(1) = Link([0 106.5  0 0],'modified');
% L(2) = Link([0 26.65  0 pi/2],'modified');
% L(2).offset = pi/2;
% L(3)= Link([0 -30.3 250 0],'modified');
% L(4)= Link([0 244.499 76.5 pi/2],'modified');
% L(5)= Link([0 0 0 pi/2],'modified');
% L(6)= Link([pi/2 0 0 -pi/2],'modified');%85.5 3
% L(6).offset = pi/2;
%% 标准型DH参数
L(1) = Link([0 106.5  0 pi/2]);
L(2) = Link([0 26.65  250 0]);
L(3)= Link([0 -30.3 76.5 pi/2]);
L(4)= Link([0 244.499 0 -pi/2]);
L(5)= Link([0 0 0 pi/2]);
L(6)= Link([0 85.5 0 0]);
%% 建立机械臂模型
instinct2=SerialLink(L,'name','instinct2');
% view(3); 
% instinct2.teach();
qn=[pi/4 pi/3 -pi/6 pi/6 -pi/6 -pi/4];
T=instinct2.fkine(qn);
t_m=double(T);
%% 解析解逆解
% 创建平移变换矩阵
d6 = -85.5;
T_translate = eye(4);
T_translate(3, 4) = d6;

% 应用平移变换
t_m = t_m * T_translate;
R = t_m(1:3, 1:3);  % 旋转矩阵 (3×3)
% disp(R)
t = t_m(1:3, 4);    % 平移向量 (3×1)

% 赋值给单独变量
x = t(1);
y = t(2);
z = t(3);

%求解位置
%首先解theta1
t11=atan2d(y,x+3.650);
t12=t11-180;
disp("theta1结果：")
disp(t11)
disp(t12)

%接着解theta3
d1=106.5;d2=250;d3=sqrt(76.5^2+244.499^2);
wa=z-d1;
wb=sqrt(x^2+y^2-3.65^2);
w=sqrt(wa^2+wb^2);
bias=atan2d(76.5,244.499);
c3=(d2^2+d3^2-w^2)/(2*d2*d3);
t3=acosd(c3);
disp("theta3结果：")
t31=t3-bias-90;
t32=360-t31-90;
disp(t31);
disp(t32);

%然后解theta2
tbase=atan2d(wa,wb);
cd3=acosd((d2^2+w^2-d3^2)/(2*d2*w));
t211=tbase+cd3;
t222=tbase-cd3;
t213=180-(tbase+cd3);
t224=180-(tbase-cd3);
disp("theta2结果：")
disp(t211)
disp(t222)
disp(t213)
disp(t224)

%共有四种可能：
disp("四组解结果：")
disp([t11, t211, t31]);
disp([t11, t222, t32]);
disp([t12, t213, t31]);
disp([t12, t224, t32]);

%比较最优解(未完成)
theta1=deg2rad(t11);
theta2=deg2rad(t211);
theta3=deg2rad(t31);


%求解姿态（ZYZ欧拉角）
function [T01, T12, T23] = calculate_dh_transforms(theta1, theta2, theta3)
    % DH参数 (单位: mm 和 rad)
    alpha1 = pi/2; a1 = 0;     d1 = 106.5;
    alpha2 = 0;     a2 = 250;  d2 = 26.65;
    alpha3 = pi/2;   a3 = 76.5; d3 = -30.3;
    
    % 计算 T01
    T01 = [
        cos(theta1), -sin(theta1)*cos(alpha1),  sin(theta1)*sin(alpha1), a1*cos(theta1);
        sin(theta1),  cos(theta1)*cos(alpha1), -cos(theta1)*sin(alpha1), a1*sin(theta1);
        0,            sin(alpha1),             cos(alpha1),             d1;
        0,            0,                        0,                       1
    ];
    
    % 计算 T12
    T12 = [
        cos(theta2), -sin(theta2)*cos(alpha2),  sin(theta2)*sin(alpha2), a2*cos(theta2);
        sin(theta2),  cos(theta2)*cos(alpha2), -cos(theta2)*sin(alpha2), a2*sin(theta2);
        0,            sin(alpha2),              cos(alpha2),              d2;
        0,            0,                        0,                        1
    ];
    
    % 计算 T23
    T23 = [
        cos(theta3), -sin(theta3)*cos(alpha3),  sin(theta3)*sin(alpha3), a3*cos(theta3);
        sin(theta3),  cos(theta3)*cos(alpha3), -cos(theta3)*sin(alpha3), a3*sin(theta3);
        0,            sin(alpha3),              cos(alpha3),             d3;
        0,            0,                        0,                        1
    ];
end

[T01, T12, T23] = calculate_dh_transforms(theta1, theta2, theta3);
T03=T01*T12*T23;
R36=transpose(T03(1:3, 1:3))*R;

[r11, r12, r13] = deal(R36(1,1), R36(1,2), R36(1,3));
[r21, r22, r23] = deal(R36(2,1), R36(2,2), R36(2,3));
[r31, r32, r33] = deal(R36(3,1), R36(3,2), R36(3,3));

if abs(r31)<0.0148&&abs(r32)<0.0148
   beta_show=0;
   alpha=0;
   gamma=atan2d(-r12,r11);
else
    beta1=atan2(-sqrt(r31^2+r32^2),r33);
    beta1_show=atan2d(-sqrt(r31^2+r32^2),r33);
    alpha1=atan2d(r23/sin(beta1),r13/sin(beta1));
    gamma1=atan2d(r32/sin(beta1),-r31/sin(beta1));
    beta2=atan2(sqrt(r31^2+r32^2),r33);
    beta2_show=atan2d(sqrt(r31^2+r32^2),r33);
    alpha2=atan2d(r23/sin(beta2),r13/sin(beta2));
    gamma2=atan2d(r32/sin(beta2),-r31/sin(beta2));
end

disp("theta4、5、6结果：")
disp([alpha1,beta1_show,gamma1])
disp([alpha2,beta2_show,gamma2])

%最终八种可能：
disp("关节空间qn：")
disp(rad2deg(qn))
disp("八组解结果：")
disp([t11, t211, t31,alpha1,beta1_show,gamma1]);
disp([t11, t222, t32,alpha1,beta1_show,gamma1]);
disp([t12, t213, t31,alpha1,beta1_show,gamma1]);
disp([t12, t224, t32,alpha1,beta1_show,gamma1]);
disp([t11, t211, t31,alpha2,beta2_show,gamma2]);
disp([t11, t222, t32,alpha2,beta2_show,gamma2]);
disp([t12, t213, t31,alpha2,beta2_show,gamma2]);
disp([t12, t224, t32,alpha2,beta2_show,gamma2]);

%% 迭代解逆解
q1=instinct2.ikine(T);
disp("迭代解：")
disp(rad2deg(q1));
instinct2.plot(q1);

