%   Description: 采用UR10机械臂，实现一个水平面的虚拟质量阻尼系统，再此平面内实现论文中观测器，以及更新率的仿真
%   Author: 唐纪元  2025/1/15
%   

clc;
clear;
%% 机械臂模型构建
baseOffsetX = 0.4;
baseOffsetY = 0.4;
baseOffset = [baseOffsetX; baseOffsetY];

mdl_ur10;
qn = [0 0 0 0 0 0];
ur10.links(1).offset= pi;
ur10.links(2).offset= -pi/2;
ur10.links(4).offset= -pi/2;
ur10.tool = SE3(0,0,0.1);
ur10.fkine(qn);
%ur10.plot(qn);

T = SE3(baseOffsetX + 0.2, baseOffsetX + 0.2, 0) * SE3.Rx(pi);       %末端执行器初始位置的变换矩阵，以0.5,0.5,0为原点，末端执行器在此平面内移动

qStart = ur10.ikine(T,'q0',[0 0 0 0 0 0]);  %末端执行器初始位置的关节向量
jointPos = zeros(1,6);
jointVel = zeros(1,6);

cartisianPos = zeros(2,1);
cartisianVel = zeros(2,1);

J = ur10.jacob0(qStart);
%% 虚拟2维质量阻尼系统 参数初始化，所有参数以UR10基坐标系[0.5,0.5,0]为原点
Tf = 80; % 仿真总时间
step = 0.01; % 步长
t = 0;   %实时仿真时间
T = Tf / step + 1; % 步数


% 在2维平面中虚拟的M，B矩阵，参数可变
M = [2  0;
     0  2];

C = [2  0;
     0  2];

% 位置变量，输出给机械臂
x = [0.2; 0.2];% 当前位置
dx = zeros(2, 1);% 当前速度
ddx = zeros(2, 1);% 当前加速度

% 参考位置变量
xr = [0.2; 0.2]; % 参考位置
dxr = zeros(2, 1); % 参考速度
ddxr = zeros(2, 1); % 参考加速度
dxr_old = 0;

% 人类想要到达的轨迹(一个目标点)
xh = [0.4; 0.45]; % 理想位置
dxh = zeros(2, 1); % 理想速度
ddxh = zeros(2, 1); % 理想加速度

% 误差系统参数
e = x - xr; % 位置误差
de = dx - dxr; % 速度误差 
dde = ddx - ddxr; % 加速度误差

xi = [e ; de];
dxi = [de ; dde];

%机器人控制器参数
Kp = 180 * eye(2);
Kd = 50 * eye(2);

K_hat = zeros(2);
Kv_hat = zeros(2);
Kp_hat = zeros(2,1);

Af = 120 * eye(2);
Ad = 100 * eye(2);
Ac = [18 0;
      0  14];

u_ff = M * ddxr + C * dxr;   %前馈控制输入
u_fb = -Kp * e - Kd * de;    %反馈控制输入
u_cp = -K_hat * x -Kv_hat * dx - Kp_hat; %补偿干扰力

%人类控制器参数
K_hp = 200 * eye(2);
K_hd = 50 * eye(2);
fh = -1 * K_hp * xh;
uh = -1 * fh - K_hp * x - K_hd * dx;  %人类控制器输入

% 观测器系统参数
A = [zeros(2)       eye(2);
     -M^-1 * Kp   -M^-1*(C + Kd)];
B = [zeros(2); 
     M^-1];

alpha = 1.5; % 更新速率x
L = [5, 0, 0, 0; 1, 4, 0, 1; 0.5, 0, 4, 0; 0, 0, 0, 5];  %观测器反馈矩阵
xi_hat = [e ; de];  %误差估计值          
dxi_hat = [de ; dde];

xi_tilde = xi_hat - xi;  %误差估计值与实际误差的偏差 
dxi_tilde = dxi_hat - dxi;

uh_hat = zeros(2,1);     %估计人类控制器输入
fh_hat = zeros(2,1);
K_hp_hat = zeros(2);
K_hd_hat = zeros(2);
d_tilde_hat = zeros(2);
uh_out = zeros(2,T);
uh_hat_out = zeros(2,T);
tracking_error = zeros(2,T);

d_out = [];
d_hat_out = [];

K = 25 *eye(2);
Kv = 25 * eye(2);
d = K * x + Kv * dx;     % 干扰力

%记录数据用
time = [];
xr_out = []; 
xh_out = []; 
x_out = [];
%%  仿真
client = RemoteAPIClient();
sim = client.require('sim');

sim.setStepping(true);
sim.startSimulation();

jointNames={'/UR10/joint1','/UR10/joint2','/UR10/joint3','/UR10/joint4','/UR10/joint5','/UR10/joint6'};
Joints = -ones(1,6); 

%获取机械臂关节句柄
for i = 1:6
    Joints(i) = sim.getObject(jointNames{i}); 
end

forceSensor = sim.getObject('/UR10/forceSensor');
endEffector = sim.getObject('/UR10/endEffector');


setPosition(sim,Joints,qStart);

while true

    t = sim.getSimulationTime();

    if t >= 3; break; end
    setPosition(sim,Joints,qStart);    
    fprintf('Simulation time: %.3f [s]\n', t);
    sim.step();
    
end
    initialTime = t;

while true

    t = sim.getSimulationTime() - initialTime;
    %读取关节角度值
    for i = 1:6
        jointPos(i) = sim.getJointPosition(Joints(i));
        jointVel(i) = sim.getJointVelocity(Joints(i));
    end


    %正运动学解算得当前末端执行器的位置
    T = ur10.fkine(jointPos);
    cartisianPos = T.t(1:2,1);
    
    J = ur10.jacob0(jointPos);
    cartisianVel = J(1:2, :) * jointVel'; % 取线速度，不要角速度

    %更新人类轨迹
    xh(1) = - exp(-t-log(5)) + 0.4 ;
    xh(2) = - exp(-t-log(4)) + 0.45;
    
    %更新机器人参考轨迹
    dxr = alpha * uh_hat;       %更新率
    xr = xr + dxr * step;               
    ddxr = (dxr - dxr_old)/step;
    dxr_old = dxr;

    e = (cartisianPos - baseOffset) - xr; % 位置误差
    de = cartisianVel - dxr; % 速度误差 
    xi = [e ; de];
    
    %更新人力以及干扰力输入
    noise1 = rand(1)-0.5;
    noise2 = rand(1)-0.5;
    fh = -1 * K_hp * xh; 
    uh = -1 * fh - K_hp * (cartisianPos - baseOffset) - K_hd * cartisianVel; 
    d = K *  (cartisianPos - baseOffset)  + Kv * cartisianVel;     % 干扰力
    sim.addForceAndTorque(endEffector, [uh(1)+d(1)+ 2*noise1, uh(2)+d(2)+2*noise2,0], zeros(1,3));  %施加力

    [result,f_ext,torque_ext] = sim.readForceSensor(forceSensor); %读取力传感器数据,传入到f_ext中
    f_ext = cellfun(@double, f_ext, 'UniformOutput', false);
    f_ext = cell2mat(f_ext');

    % 交换 f_ext 的x和y数据，传感器坐标显示问题
    temp = f_ext(1);
    f_ext(1) = f_ext(2);
    f_ext(2) = temp;
    
    %控制输入
    uff  = M * ddxr + C * dxr;
    u_fb = -Kp * e - Kd * de;
    u_cp = -( K_hat *  (cartisianPos - baseOffset) + Kv_hat * cartisianVel + Kp_hat); %补偿干扰力 
    u = uff ;
    
    %估计干扰力d方便在输出中进行抵消
    K_hat = K_hat + (de  - B'*xi_tilde)* (cartisianPos - baseOffset)' * Af * step;
    Kv_hat = Kv_hat + (de - B'*xi_tilde)* cartisianVel' * Ad * step;
    Kp_hat = Kp_hat + Ac * (de - B'*xi_tilde) * step;
    d_hat = K_hat * (cartisianPos - baseOffset) + Kv_hat * cartisianVel + Kp_hat; 
    
    % 观测器
    dxi_hat = A * xi_hat + B * uh_hat - L * xi_tilde;
    dfh_hat = B'*xi_tilde + alpha * uh_hat;
    dK_hp_hat = (B'*xi_tilde + alpha * uh_hat) * (cartisianPos - baseOffset)';
    dK_hd_hat = (B'*xi_tilde + alpha * uh_hat) * cartisianVel';
    
    xi_hat = xi_hat + dxi_hat * step;    
    fh_hat = fh_hat + dfh_hat * step;   
    K_hp_hat = K_hp_hat + dK_hp_hat * step;
    K_hd_hat = K_hd_hat + dK_hd_hat * step; 
    xi_tilde = xi_hat - xi;
    uh_hat = - fh_hat - K_hp_hat * (cartisianPos - baseOffset) - K_hd_hat * cartisianVel;
    
    %计算目标位置
    ddx = M^-1 *(u + f_ext(1:2) - C * cartisianVel);
    dx =  cartisianVel + ddx * step;
    cartisianPosNew = cartisianPos + dx * step;
    
    T.t(1) = cartisianPosNew(1);
    T.t(2) = cartisianPosNew(2);
    
    q  = ur10.ikine(T,'q0',[0 0 0 0 0 0]);
    
    setPosition(sim,Joints,q);
    
    
    if t >= Tf; break; end
    fprintf('Simulation time: %.3f [s]\n', t);
    sim.step();

    % 记录参数
    time = [time,t];
    x_out = [x_out,cartisianPosNew - baseOffset];
    xr_out = [xr_out,xr];
    xh_out = [xh_out,xh];
    d_out = [d_out,d];
    d_hat_out = [d_hat_out,d_hat];

end

sim.stopSimulation();

%% 打印参数
figure(1);
plot(time, xr_out(1, :),'r--','linewidth',2);
hold on;
plot(time, x_out(1, :),'b','linewidth',2);
hold on;
plot(time,xh_out(1, :),'g','linewidth',2);
axis()
legend('参考轨迹','当前轨迹','人类目标轨迹','FontName','宋体','FontSize',18,'FontWeight','bold');
ylim([0.15 0.5]);
xlabel('时间(s)','FontName','宋体','FontSize',18,'FontWeight','bold')
ylabel('x轴位置(m)','FontName','宋体','FontSize',18,'FontWeight','bold')
grid;

figure(2);
plot(time, xr_out(2, :),'r--','linewidth',2);
hold on;
plot(time, x_out(2, :),'b','linewidth',2);
hold on;
plot(time,xh_out(2, :),'g','linewidth',2);
legend('参考轨迹','当前轨迹','人类目标轨迹','FontName','宋体','FontSize',18,'FontWeight','bold');
ylim([0.15 0.5]);
xlabel('时间(s)','FontName','宋体','FontSize',18,'FontWeight','bold')
ylabel('y轴位置(m)','FontName','宋体','FontSize',18,'FontWeight','bold')

grid;

figure(3);
plot(time, d_out(1, :),'b','linewidth',2);
hold on;
plot(time, d_hat_out(1, :),'r-','linewidth',2);
legend( '实际干扰力','估计干扰力','FontName','宋体','FontSize',18,'FontWeight','bold');
xlabel('时间(s)','FontName','宋体','FontSize',18,'FontWeight','bold')
ylabel('x轴干扰力(N)','FontName','宋体','FontSize',18,'FontWeight','bold')

grid;

figure(4);
plot(time, d_out(2, :),'b','linewidth',2);
hold on;
plot(time, d_hat_out(2, :),'r-','linewidth',2);
legend( '实际干扰力','估计干扰力','FontName','宋体','FontSize',18,'FontWeight','bold');
xlabel('时间(s)','FontName','宋体','FontSize',18,'FontWeight','bold')
ylabel('y轴干扰力(N)','FontName','宋体','FontSize',18,'FontWeight','bold')

grid;
% figure(3);
% plot(time, tracking_error(1, :),'r--');
% legend('xh1');
% xlabel('Time(s)')
% ylabel('Error(m)')
% grid;
% 
% figure(4);
% plot(time, tracking_error(2, :),'r--');
% legend('xh2');
% xlabel('Time(s)')
% ylabel('Error(m)')
% grid;
