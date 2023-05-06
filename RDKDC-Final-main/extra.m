ur5=ur5_interface();
g1=[0,-1,0,-0.085;-1,0,0,0.65;0,0,-1,0.085;0,0,0,1];
g1_above=g1+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g1_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,10);
pause(10);

q=ur5InvKin(g1);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g2=g1;
g2(1:2,4)=g2(1:2,4)+[0.04;-0.03];
q=ur5InvKin(g2);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,3);
pause(3);

g2_above=g2+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g2_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

%z
g3=g2;
g3(1:2,4)=g3(1:2,4)+[-0.12;0];
g3_above=g3+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g3_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,5);
pause(5);

q=ur5InvKin(g3);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g4=g3;
g4(1:2,4)=g4(1:2,4)+[0.02;-0.05];
q=ur5InvKin(g4);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,3);
pause(3);

g4_above=g4+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g4_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

%z
g5=g4;
g5(1:2,4)=g5(1:2,4)+[-0.01;0.025];
g5_above=g5+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g5_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

q=ur5InvKin(g5);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g6=g5;
g6(1:2,4)=g6(1:2,4)+[0.21;0.025];
q=ur5InvKin(g6);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,10);
pause(10);

g7=g6;
g7(1:2,4)=g7(1:2,4)+[-0.04;-0.05];
q=ur5InvKin(g7);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,4);
pause(4);

g7_above=g7+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g7_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

%^z
g8=g7;
g8(1:2,4)=g8(1:2,4)+[-0.12;-0.03];
g8_above=g8+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g8_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,5);
pause(5);

q=ur5InvKin(g8);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g9=g8;
g9(1:2,4)=g9(1:2,4)+[0.07;0.005];
q=ur5InvKin(g9);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,2);
pause(2);

g9_above=g9+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g9_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

%^z
g10=g9;
g10(1:2,4)=g10(1:2,4)+[-0.015;0.025];
g10_above=g10+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g10_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

q=ur5InvKin(g10);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g11=g10;
g11(1:2,4)=g11(1:2,4)+[-0.03;-0.05];
q=ur5InvKin(g11);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,4);
pause(4);

g11_above=g11+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g11_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

%z
g12=g11;
g12(1:2,4)=g12(1:2,4)+[0.015;0.025];
g12_above=g12+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g12_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

q=ur5InvKin(g12);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g13=g12;
g13(1:2,4)=g13(1:2,4)+[0.015;-0.025];
q=ur5InvKin(g13);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g13_above=g13+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g13_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

%z
g14=g13;
g14(1:2,4)=g14(1:2,4)+[-0.12;-0.05];
g14_above=g14+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g14_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,5);
pause(5);

q=ur5InvKin(g14);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g15=g14;
g15(1:2,4)=g15(1:2,4)+[0.24;0.02];
q=ur5InvKin(g15);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,10);
pause(10);

g15_above=g15+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g15_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

%z
g16=g15;
g16(1:2,4)=g16(1:2,4)+[-0.03;-0.0025];
g16_above=g16+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g16_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

q=ur5InvKin(g16);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g17=g16;
g17(1:2,4)=g17(1:2,4)+[0;-0.18];
q=ur5InvKin(g17);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,10);
pause(10);

g18=g17;
g18(1:2,4)=g18(1:2,4)+[-0.03;0.03];
q=ur5InvKin(g18);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,3);
pause(3);

g18_above=g18+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g18_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

%z
g19=g18;
g19(1:2,4)=g19(1:2,4)+[-0.09;0.09];
g19_above=g19+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g19_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,5);
pause(5);

q=ur5InvKin(g19);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g20=g19;
g20(1:2,4)=g20(1:2,4)+[0;-0.06];
q=ur5InvKin(g20);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,2);
pause(2);

g20_above=g20+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g20_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

%z
g21=g20;
g21(1:2,4)=g21(1:2,4)+[0;0.06];
g21_above=g21+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g21_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,2);
pause(2);

q=ur5InvKin(g21);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g22=g21;
g22(1:2,4)=g22(1:2,4)+[0.06;0];
q=ur5InvKin(g22);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g23=g22;
g23(1:2,4)=g23(1:2,4)+[0;-0.06];
q=ur5InvKin(g23);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,2);
pause(2);

g23_above=g23+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g23_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);


g24=g23;
g24(1:2,4)=g24(1:2,4)+[-0.06;0];

g24_above=g24+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g24_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,2);
pause(2);

q=ur5InvKin(g24);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

g25=g24;
g25(1:2,4)=g25(1:2,4)+[0.06;0];

q=ur5InvKin(g25);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,2);
pause(2);

g25_above=g25+[0,0,0,0;0,0,0,0;0,0,0,0.02;0,0,0,0];
q=ur5InvKin(g25_above);
q_chosen=q(:,1)
ur5.move_joints(q_chosen,1);
pause(1);

display("achieved")