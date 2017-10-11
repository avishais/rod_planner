% FK For baxter robot
% Steve Macenski (c) 2017

function [T] = BaxterFK(angle)
    % insert array of 1xn angles in degrees
    % return T cell array

    base_t = 1000*[0;0;0];
    l1_tR = 1000*[0.024645+0.055695-0.02; -0.25; 0.118588+0.011038];
    l2_tR = 1000*[0.073; 0; 0.245]; 
    l3_tR = 1000*[0.102; 0; 0]; 
    l4_tR = 1000*[0.069; 0; 0.26242-0.015]; 
    l5_tR = 1000*[0.10; 0; 0]; 
    l6_tR = 1000*[0.01; 0; 0.2707]; 
    l7_tR = 1000*[0.16; 0; 0];
    lEE_tR = 1000*[0; 0; 0.05];

    R00 = eye(3);
    R01 = R00 * rotz(angle(1)); 
    R02 = R01 * roty(angle(2)) * rotx(-90);
    R03 = R02 * rotx(angle(3) + 90) * roty(90);
    R04 = R03 * roty(angle(4)) * rotx(90);
    R05 = R04 * rotx(angle(5) - 90) * roty(90);
    R06 = R05 * roty(angle(6) - 90) * rotx(90);
    R07 = R06 * rotx(angle(7) + 90) * roty(90);
    R0EE = R07;

    t1 = R00 * l1_tR + base_t;
    t2 = R01 * l2_tR + t1;
    t3 = R02 * l3_tR + t2;
    t4 = R03 * l4_tR + t3;
    t5 = R04 * l5_tR + t4;
    t6 = R05 * l6_tR + t5;
    t7 = R06 * l7_tR + t6;
   tEE = R07 * lEE_tR + t7;
    
    T01 = eye(4); T01(1:3,1:3) = R01; T01(1:3,4) = t1;
    T02 = eye(4); T02(1:3,1:3) = R02; T02(1:3,4) = t2;
    T03 = eye(4); T03(1:3,1:3) = R03; T03(1:3,4) = t3;
    T04 = eye(4); T04(1:3,1:3) = R04; T04(1:3,4) = t4;
    T05 = eye(4); T05(1:3,1:3) = R05; T05(1:3,4) = t5;
    T06 = eye(4); T06(1:3,1:3) = R06; T06(1:3,4) = t6;
    T07 = eye(4); T07(1:3,1:3) = R07; T07(1:3,4) = t7;
    T0EE = eye(4);T0EE(1:3,1:3) = R0EE; T0EE(1:3,4) = tEE; 
    
    T = {T01, T02, T03, T04, T05, T06, T07, T0EE};
end