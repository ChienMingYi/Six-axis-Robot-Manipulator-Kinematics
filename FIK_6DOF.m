X = 80;    Y = 10;    Z = 20;

rpy = [0 0 0]*1;  % static rot from right to left (yaw->pitch->roll) = (x->y->z)


Roll  = rpy(1);   % rot global z-axis (b)
Pitch = rpy(2);   % rot global y-axis (g)
Yaw   = rpy(3);   % rot global x-axis (r)
JointAngle = [];

FIK_6DOF(X, Y, Z, Roll, Pitch, Yaw);   % 初始化手臂
clf;
for i = [0:10:360]
    FIK_6DOF(X, Y, Z, Roll, Pitch, Yaw);
end
function FIK_6DOF(X, Y, Z, Roll, Pitch, Yaw)
%% 【Step １】 建立各種參數資訊
%% 1.1 定義手臂自由度與DH連桿參數表
% 手臂的自由度
DOF = 6;     
% 六軸的 DH 參數  [ a     α        d       θ ]
DH_Parameter = [   0     pi/2     10        0; 
                  60        0      0        0; 
                   0     pi/2      0     pi/2; 
                   0    -pi/2     50        0;
                   0     pi/2      0        0; 
                   0        0     20        0   ];  
              
Pos = [ X ; Y; Z ];
Euler_RPY = [Roll, Pitch, Yaw];

%% 【Step 2】 計算逆運動學
JointAngle = InverseKinematics(Pos, Euler_RPY, DOF, DH_Parameter);
JointAngle*180/pi
%% 【Step 3】 根據逆運動學結果，代入正運動學驗證
Info = ForwardKinematics( DOF, JointAngle, DH_Parameter );

%% 【Step 4】 繪製手臂
DrawRobotManipulator( DOF, Info.JointPos, Info.JointDir );  

%% 【Step 5】 顯示正運動學得出之資訊
figure(1);
str_x = num2str(roundn(Info.P(1), -2));
str_y = num2str(roundn(Info.P(2), -2));
str_z = num2str(roundn(Info.P(3), -2));
str_Pitch = num2str(roundn(Info.Pitch*180/pi, -2));
str_Roll  = num2str(roundn(Info.Roll*180/pi , -2));
str_Yaw   = num2str(roundn(Info.Yaw*180/pi  , -2));

str = [  'X Y Z = ' str_x,     '   ',  str_y, '   ',    str_z, '     '...
         'R P Y = ' str_Roll, '   ',  str_Pitch, '   ', str_Yaw] ;

title(str)
end

% =========================================================================
%                              【Functions】 
% =========================================================================

%% 逆運動學
% 輸入：末端點位置、末端點姿態、自由度、DH連桿表 
% 輸出：各關節的角度
function JointAngle = InverseKinematics(Pos, Euler_RPY, DOF, DH_Parameter)
    if(abs(Euler_RPY(2))>90)
        fprintf('*** note: pitch>90 degree, feed back會差一個pi *** \n')
    end
    DesiredCmd.Roll  = Euler_RPY(1) * pi/180;
    DesiredCmd.Pitch = Euler_RPY(2) * pi/180;
    DesiredCmd.Yaw   = Euler_RPY(3) * pi/180;
    DesiredCmd;
    %% Step 1. 建立姿態矩陣 
    a  = DesiredCmd.Roll;
    b  = DesiredCmd.Pitch;
    c  = DesiredCmd.Yaw;
    RX = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];% RX
    RY = [cos(b) 0 sin(b); 0 1 0;-sin(b) 0 cos(b)]; % RY
    RZ = [cos(c) -sin(c) 0; sin(c) cos(c) 0;0 0 1]; % RZ
    OrienMat = RZ*RY*RX;  
    %--------------------------------------------------------------------------
    %% Step 2. 初始化相關參數
    DesiredCmd.P     = Pos;                          % desired position
    DesiredCmd.R     = OrienMat;                     % desired orientation
    DesiredCmd.Elbow = -1;                           % Elbow Up = -1, Elbow Down =  1
    DesiredCmd.Wrist = 1;                           % Wrist Up =  1, Wrist Down = -1
    DesiredCmd.L1    = DH_Parameter( 1, 3 );
    DesiredCmd.L2    = DH_Parameter( 2, 1 ); 
    DesiredCmd.L3    = DH_Parameter( 4, 3 );
    DesiredCmd.L4    = DH_Parameter( 6, 3 );         % d3 一般而言就是夾具長度
    DesiredCmd.Angle = zeros( 1, DOF );

    %% Step 3.  逆位置運動學
    DesiredCmd = InversePosition( DesiredCmd ); 

    %% Step 4.  逆姿態運動學              
    DesiredCmd = InverseOrientation( DesiredCmd, DH_Parameter );
    
    %% Step 5.  輸出計算結果
    JointAngle = DesiredCmd.Angle;
end

%% 逆位置運動學
% 輸入：末端點位置、末端點姿態、自由度、DH連桿表 
% 輸出：第1~3軸的角度
function Cmd = InversePosition( Cmd )
    WristPos = Cmd.P - Cmd.L4 * Cmd.R( 1:3, 3 );  %Oc = [Xc ; Yc ; Zc] = [WristPos(1) WristPos(2) WristPos(3)]

    if( abs( WristPos(1) ) < 0.0001 && abs( WristPos(2) ) < 0.0001 )
        Cmd.Angle(1) = 0;
    else
        Cmd.Angle(1) = atan2( WristPos(2) , WristPos(1) );% Yc/Xc
    end
    xc_2 = WristPos(1)^2;
    yc_2 = WristPos(2)^2;

    s    = WristPos(3) - Cmd.L1;
    D    = acos(( Cmd.L2^2 + Cmd.L3^2 - xc_2 - yc_2 - s^2  )/( 2.0 * Cmd.L2 * Cmd.L3 ));
    Cmd.Angle(3)= D - pi;
    Cmd.Angle(2) = atan2( s, sqrt( xc_2 + yc_2 ) ) - atan2(Cmd.L3*sin( Cmd.Angle(3) ), Cmd.L2 + Cmd.L3*cos( Cmd.Angle(3) ));
end

%% 逆姿態運動學
% 輸入：末端點位置、末端點姿態、自由度、DH連桿表 
% 輸出：第4~6軸的角度
function Cmd = InverseOrientation( Cmd, DH_Parameter )
    s1  = sin( Cmd.Angle(1) + DH_Parameter(1,4) );
    c1  = cos( Cmd.Angle(1) + DH_Parameter(1,4) );
    s23 = sin( Cmd.Angle(2) + DH_Parameter(2,4) + Cmd.Angle(3) + DH_Parameter(3,4) );
    c23 = cos( Cmd.Angle(2) + DH_Parameter(2,4) + Cmd.Angle(3) + DH_Parameter(3,4) );

    R0_3 = [    c1*c23    s1   c1*s23;
                s1*c23   -c1   s1*s23;
                   s23     0     -c23  ];
    R3_6 = R0_3' * Cmd.R;

    Cmd.Angle(5) = atan2( Cmd.Wrist * sqrt( 1 - R3_6(3,3)^2 ), R3_6(3,3) );

    if( abs( R3_6(3,3) ) > 0.9999 )

        Cmd.Angle(4) = 0;
        Cmd.Angle(6) = atan2( R3_6(2,1) , R3_6(1,1) );

    else
        if( Cmd.Wrist > 0 )
            Cmd.Angle(4) = atan2( R3_6(2,3) ,  R3_6(1,3) );
            Cmd.Angle(6) = atan2( R3_6(3,2) , -R3_6(3,1) );     

        else
            Cmd.Angle(4) = atan2( -R3_6(2,3) , -R3_6(1,3) );
            Cmd.Angle(6) = atan2( -R3_6(3,2) ,  R3_6(3,1) );  

        end
    end
end

%% 正運動學推導函式
% 輸入：自由度、各關節的角度、DH連桿表 
% 輸出：各關節的位置、各關節的座標
function Info = ForwardKinematics( DOF, JointAngle, DH_Parameter )
    %% Step 1.初始化各關節位置向量與姿態矩陣
    Info.JointPos = [0 ;0 ;0];                              % 各關節的座標位置矩陣 [ 3 x n ] , n = DOF
    Info.JointDir = [1 0 0; 0 1 0; 0 0 1];                  % 各關節的座標向量矩陣 [ 3 x 3n ], n = DOF
    Info.T0_6     = [ 1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];  % 由正運動學求出的轉移函數，T0_6 表示是從 Frame0 ~ Frame6

    %% Step 2.使用齊次轉換矩陣求解正運動學
    for i = 1 : DOF
        A        = GenerateTransformationMatrices( JointAngle(i) , DH_Parameter(i,:) ); % 依據各軸的DH參數代入DH轉移函數中
        Info.T0_6     = Info.T0_6 * A;                                                  % 將其作連乘的動作
        Info.JointPos = [ Info.JointPos Info.T0_6( 1:3, 4 ) ];                          % 儲存關節的座標位置
        Info.JointDir = [ Info.JointDir Info.T0_6( 1:3, 1:3 ) ];                        % 儲存關節的向量資訊
    end
     % 依據Rotation 矩陣 求解 Pitch, Roll, Yaw

    %% Step 3.擷取末端點位置向量與姿態矩陣
    Info.P = Info.T0_6( 1:3, 4 );
    Info.R = Info.T0_6( 1:3, 1:3 );
    R0_6   = Info.T0_6( 1:3, 1:3 );

    %% Step 4.分析末端點的姿態(Pitch Roll Yaw) 
    cal_err = 1*10^-8;
    if( abs(R0_6(3,1)-1) < cal_err)         
        Info.Yaw   = 0;
        Info.Pitch = -pi/2;
        Info.Roll  = atan2(-R0_6(1,2), -R0_6(1,3));


    elseif( abs(R0_6(3,1)+1) < cal_err )    
        Info.Yaw   = 0;
        Info.Pitch = pi/2;
        Info.Roll  = atan2(-R0_6(1,2), R0_6(1,3));

    else                                   
        Info.Roll  = atan2(R0_6(3,2), R0_6(3,3));
        Info.Pitch = asin(-R0_6(3,1));
        Info.Yaw   = atan2(R0_6(2,1), R0_6(1,1));
    end
    % 正規化角度(介於正負180度之間)
    Info.Roll  = normalize(Info.Roll);
    Info.Pitch = normalize(Info.Pitch);
    Info.Yaw   = normalize(Info.Yaw);

    
end

function ang = normalize(ang)
    while(ang > pi)
        ang = ang - 2*pi;
    end
    while(ang < -pi)
        ang = ang + 2*pi;
    end
end

%%  產生 轉移函數
function A = GenerateTransformationMatrices( Theta, DH_Parameter )
%    Theta = 0;
    C_Theta = cos( Theta + DH_Parameter(4) );
    S_Theta = sin( Theta + DH_Parameter(4) );
    C_Alpha = cos( DH_Parameter(2) );
    S_Alpha = sin( DH_Parameter(2) ); 
    
    A = [   C_Theta   -1*S_Theta*C_Alpha        S_Theta*S_Alpha     DH_Parameter(1) * C_Theta; 
            S_Theta      C_Theta*C_Alpha     -1*C_Theta*S_Alpha     DH_Parameter(1) * S_Theta;
            0                    S_Alpha                C_Alpha                DH_Parameter(3);
            0                          0                      0                             1   ];
    
end
%%  畫圖 畫機械手臂 輸入: 自由度，各關節的位置，各關節的座標
function DrawRobotManipulator( DOF, JointPos, JointDir  )
    % figure(1)
    % hold off
    plot3( JointPos(1,1:end), JointPos(2,1:end), JointPos(3,1:end),'linewidth', 4)
    hold on
    plot3( JointPos(1,1:end), JointPos(2,1:end), JointPos(3,1:end),'ro','linewidth', 7);

    grid on
    xlabel('X軸');
    ylabel('Y軸');
    zlabel('Z軸');
    X =10;
    Y =10;
    Z =20;
    %-------------------------------------
    BaseX = [X X -X -X X];
    BaseY = [Y -Y -Y Y Y];
    BaseZ = [0 0 0 0 0];
    patch(BaseX,BaseY,BaseZ,'k')
    %-------------------------------------
    BaseX = [X X X X X];
    BaseY = [Y -Y -Y Y Y];
    BaseZ = [0 0 -Z -Z 0];
    patch(BaseX,BaseY,BaseZ,'k')
    %-------------------------------------
    BaseX = [X -X -X X X];
    BaseY = [Y Y Y Y Y];
    BaseZ = [0 0 -Z -Z 0];
    patch(BaseX,BaseY,BaseZ,'k')
    %-------------------------------------
    BaseX = [-X -X -X -X -X];
    BaseY = [-Y Y Y -Y -Y];
    BaseZ = [0 0 -Z -Z 0];
    patch(BaseX,BaseY,BaseZ,'k')
    %-------------------------------------
    BaseX = [X -X -X X X];
    BaseY = [-Y -Y -Y -Y -Y];
    BaseZ = [0 0 -Z -Z 0];
    patch(BaseX,BaseY,BaseZ,'k')

    for i = 1 : DOF+1
         if(i==DOF+1) %只畫TCP的XYZ軸
            %-----------------------------------------
            nUnit_v = JointPos( : , i ) + 10 * JointDir( : , 3 * (i-1) + 1 );
            nBase   = [JointPos( : , i ) nUnit_v];
            plot3(nBase(1,1:end),nBase(2,1:end),nBase(3,1:end),'r','linewidth',2);
            %--------------------------------------------
            sUnit_v = JointPos( : , i ) + 10 * JointDir( : , 3 * (i-1) + 2 );
            sBase   = [JointPos( : , i ) sUnit_v];
            plot3(sBase(1,1:end),sBase(2,1:end),sBase(3,1:end),'g','linewidth',2);
            %--------------------------------------------
            aUnit_v = JointPos( : , i ) + 10 * JointDir( : , 3 * (i-1) + 3 );
            aBase   = [JointPos( : , i ) aUnit_v];
            plot3(aBase(1,1:end),aBase(2,1:end),aBase(3,1:end),'c','linewidth',2);
            %--------------------------------------------
            hold on
         end
    end

    axis equal

end 
