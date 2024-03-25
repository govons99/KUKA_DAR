#include "controller/kuka_utilities.h"


//Direct kinematic
Eigen::Vector3d D_kin(Kuka_Vec q)
{
    double C1,C2,C3,C4,C5,C6,C7;
    double S1,S2,S3,S4,S5,S6,S7;
    double l1,l2,l3,l4;
    Eigen::Matrix4d M1(4,4),M2(4,4),M3(4,4),M4(4,4),M5(4,4),M6(4,4),M7(4,4),MEE(4,4);
    Eigen::Vector3d P;

    C1=cos(q(0));
    S1=sin(q(0));
    C2=cos(q(1));
    S2=sin(q(1));
    C3=cos(q(2));
    S3=sin(q(2));
    C4=cos(q(3));
    S4=sin(q(3));
    C5=cos(q(4));
    S5=sin(q(4));
    C6=cos(q(5));
    S6=sin(q(5));
    C7=cos(0.0);
    S7=sin(0.0);

    l1=0.0; // for real robot where the base frame is on the second joint
    l2=0.4;
    l3=0.39;
    l4=0.078; // EE in the tip of KUKA without auxiliary addition
    
    M1<<C1,0,S1,0,S1,0,-C1,0,0,1,0,l1,0,0,0,1;
    M2<<C2,0,-S2,0,S2,0,C2,0,0,-1,0,0,0,0,0,1;
    M3<<C3,0,-S3,0,S3,0,C3,0,0,-1,0,l2,0,0,0,1;
    M4<<C4,0,S4,0,S4,0,-C4,0,0,1,0,0,0,0,0,1;
    M5<<C5,0,S5,0,S5,0,-C5,0,0,1,0,l3,0,0,0,1;
    M6<<C6,0,-S6,0,S6,0,C6,0,0,-1,0,0,0,0,0,1;
    M7<<C7,-S7,0,0,S7,C7,0,0,0,0,1,l4,0,0,0,1;

    MEE=(((((M1*M2)*M3)*M4)*M5)*M6)*M7;

    P=MEE.block<3,1>(0,3);
    return P;
}

//return Jacobian
Eigen::MatrixXd Jacobian(Kuka_Vec q)
{
    Eigen::MatrixXd JEE(3,6);
    double C1,S1,C2,S2,C3,S3,C4,S4,C5,S5,C6,S6;
    double l1,l2,l3,l4;

C1=cos(q(0));
S1=sin(q(0));
C2=cos(q(1));
S2=sin(q(1));
C3=cos(q(2));
S3=sin(q(2));
C4=cos(q(3));
S4=sin(q(3));
C5=cos(q(4));
S5=sin(q(4));
C6=cos(q(5));
S6=sin(q(5));

l1=0.0; // for real robot where the base frame is on the second joint
l2=0.4;
l3=0.39;
l4=0.078; // EE in the tip of KUKA without auxiliary addition

JEE(0,0)=l2*S1*S2 - l3*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) - l4*(C6*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) - S6*(C5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) + S5*(C1*C3 - C2*S1*S3)));
JEE(1,0)=- l4*(C6*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - S6*(C5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - l3*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - l2*C1*S2;
JEE(2,0)=0;

JEE(0,1)= - l4*C1*(S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5) + C6*(C2*C4 + C3*S2*S4)) - l3*C1*(C2*C4 + C3*S2*S4) - l2*C1*C2;
JEE(1,1)= - l4*S1*(S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5) + C6*(C2*C4 + C3*S2*S4)) - l3*S1*(C2*C4 + C3*S2*S4) - l2*C2*S1;
JEE(2,1)= l3*C2*C3*S4 - l3*C4*S2 - l2*S2 - l4*C4*C6*S2 + l4*C2*C3*C6*S4 + l4*C2*S3*S5*S6 - l4*C5*S2*S4*S6 - l4*C2*C3*C4*C5*S6;


JEE(0,2)= l4*C1*C2*C3*S5*S6 - l3*C1*C2*S3*S4 - l4*C3*C6*S1*S4 - l4*S1*S3*S5*S6 - l4*C1*C2*C6*S3*S4 - l3*C3*S1*S4 + l4*C3*C4*C5*S1*S6 + l4*C1*C2*C4*C5*S3*S6;
JEE(1,2)= l3*C1*C3*S4 + l4*C1*C3*C6*S4 - l3*C2*S1*S3*S4 + l4*C1*S3*S5*S6 - l4*C1*C3*C4*C5*S6 - l4*C2*C6*S1*S3*S4 + l4*C2*C3*S1*S5*S6 + l4*C2*C4*C5*S1*S3*S6;
JEE(2,2)=-S2*(l3*S3*S4 + l4*C6*S3*S4 - l4*C3*S5*S6 - l4*C4*C5*S3*S6);


JEE(0,3)= (C1*C3 - C2*S1*S3)*(l4*(S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5) + C6*(C2*C4 + C3*S2*S4)) + l3*(C2*C4 + C3*S2*S4)) + S2*S3*(l4*(C6*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) - S6*(C5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) + l3*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2));
JEE(1,3)= (C3*S1 + C1*C2*S3)*(l4*(S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5) + C6*(C2*C4 + C3*S2*S4)) + l3*(C2*C4 + C3*S2*S4)) + S2*S3*(l4*(C6*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - S6*(C5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) + l3*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2));
JEE(2,3)= l3*C3*C4*S2 - l3*C2*S4 - l4*C2*C6*S4 + l4*C3*C4*C6*S2 + l4*C2*C4*C5*S6 + l4*C3*C5*S2*S4*S6;


JEE(0,4)= l4*S6*(C3*C5*S1 + C1*C2*C5*S3 + C1*S2*S4*S5 - C4*S1*S3*S5 + C1*C2*C3*C4*S5);
JEE(1,4)= l4*S6*(C2*C5*S1*S3 - C1*C3*C5 + C1*C4*S3*S5 + S1*S2*S4*S5 + C2*C3*C4*S1*S5);
JEE(2,4)= l4*S6*(C5*S2*S3 - C2*S4*S5 + C3*C4*S2*S5);


JEE(0,5)= l4*(S5*(C2*S4 - C3*C4*S2) - C5*S2*S3)*(C6*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) - S6*(C5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) + l4*(S5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))*(S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5) + C6*(C2*C4 + C3*S2*S4));
JEE(1,5)= l4*(S5*(C2*S4 - C3*C4*S2) - C5*S2*S3)*(C6*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - S6*(C5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) + l4*(S5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))*(S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5) + C6*(C2*C4 + C3*S2*S4));
JEE(2,5)= l4*C2*C5*C6*S4 - l4*C2*C4*S6 - l4*C3*S2*S4*S6 + l4*C6*S2*S3*S5 - l4*C3*C4*C5*C6*S2;

   return JEE;

}

Eigen::MatrixXd diff_Jacobian(Kuka_Vec q, Kuka_Vec qdot)
{
    Eigen::MatrixXd dJ(3,6);
    double c1,s1,c2,s2,c3,s3,c4,s4,c5,s5,c6,s6;
    double l2,l4,l7;
    double qdot1,qdot2,qdot3,qdot4,qdot5,qdot6;

    c1=cos(q(0));
    s1=sin(q(0));
    c2=cos(q(1));
    s2=sin(q(1));
    c3=cos(q(2));
    s3=sin(q(2));
    c4=cos(q(3));
    s4=sin(q(3));
    c5=cos(q(4));
    s5=sin(q(4));
    c6=cos(q(5));
    s6=sin(q(5));

//l1=0.0; // for real robot where the base frame is on the second joint
    l2=0.4;
    l4=0.39;
    l7=0.078; // EE in the tip of KUKA without auxiliary addition

    qdot1=qdot(0);
    qdot2=qdot(1);
    qdot3=qdot(2);
    qdot4=qdot(3);
    qdot5=qdot(4);
    qdot6=qdot(5);

    dJ<<qdot1*(l4*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)+l7*(c6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)-s6*(c5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+s5*(c3*s1+c1*c2*s3)))+l2*c1*s2)-qdot4*(l7*(c6*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+c5*s6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2))+l4*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4))-qdot3*(l7*(s6*(s5*(c1*s3+c2*c3*s1)-c4*c5*(c1*c3-c2*s1*s3))+c6*s4*(c1*c3-c2*s1*s3))+l4*s4*(c1*c3-c2*s1*s3))+qdot2*(l7*(s1*s6*(s2*s3*s5+c2*c5*s4-c3*c4*c5*s2)+c6*s1*(c2*c4+c3*s2*s4))+l4*s1*(c2*c4+c3*s2*s4)+l2*c2*s1)+l7*qdot6*(s6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)+c6*(c5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+s5*(c1*c3-c2*s1*s3)))-l7*qdot5*s6*(s5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)-c5*(c1*c3-c2*s1*s3)),qdot4*(l4*c1*(c2*s4-c3*c4*s2)+l7*c1*(c6*(c2*s4-c3*c4*s2)-c5*s6*(c2*c4+c3*s2*s4)))+qdot2*(l4*c1*(c4*s2-c2*c3*s4)+l2*c1*s2+l7*c1*(s6*(c5*(s2*s4+c2*c3*c4)-c2*s3*s5)+c6*(c4*s2-c2*c3*s4)))+qdot1*(l4*s1*(c2*c4+c3*s2*s4)+l2*c2*s1+l7*s1*(s6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)+c6*(c2*c4+c3*s2*s4)))-qdot3*(l7*c1*(s6*(c3*s2*s5+c4*c5*s2*s3)-c6*s2*s3*s4)-l4*c1*s2*s3*s4)-l7*qdot6*c1*(c6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)-s6*(c2*c4+c3*s2*s4))+l7*qdot5*c1*s6*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3),l7*qdot6*(c3*s1*s4*s6-c6*s1*s3*s5+c1*c2*c3*c6*s5+c3*c4*c5*c6*s1+c1*c2*s3*s4*s6+c1*c2*c4*c5*c6*s3)-qdot3*(l4*c1*c2*c3*s4-l4*s1*s3*s4-l7*c6*s1*s3*s4+l7*c3*s1*s5*s6+l7*c1*c2*c3*c6*s4+l7*c1*c2*s3*s5*s6+l7*c4*c5*s1*s3*s6-l7*c1*c2*c3*c4*c5*s6)-qdot4*(c3*s1+c1*c2*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-qdot1*(l4*c1*c3*s4+l7*c1*c3*c6*s4-l4*c2*s1*s3*s4+l7*c1*s3*s5*s6-l7*c1*c3*c4*c5*s6-l7*c2*c6*s1*s3*s4+l7*c2*c3*s1*s5*s6+l7*c2*c4*c5*s1*s3*s6)+qdot2*c1*s2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)-l7*qdot5*s6*(c5*s1*s3-c1*c2*c3*c5+c3*c4*s1*s5+c1*c2*c4*s3*s5),qdot4*(l4*c1*c4*s2+l4*s1*s3*s4-l4*c1*c2*c3*s4+l7*c1*c4*c6*s2+l7*c6*s1*s3*s4-l7*c1*c2*c3*c6*s4+l7*c1*c5*s2*s4*s6-l7*c4*c5*s1*s3*s6+l7*c1*c2*c3*c4*c5*s6)-qdot1*(l4*c1*c4*s3+l4*s1*s2*s4+l4*c2*c3*c4*s1+l7*c1*c4*c6*s3+l7*c6*s1*s2*s4+l7*c2*c3*c4*c6*s1-l7*c4*c5*s1*s2*s6+l7*c1*c5*s3*s4*s6+l7*c2*c3*c5*s1*s4*s6)-qdot3*(c3*s1+c1*c2*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-l7*qdot6*(c1*s2*s4*s6-c4*s1*s3*s6+c1*c2*c3*c4*s6+c1*c4*c5*c6*s2+c5*c6*s1*s3*s4-c1*c2*c3*c5*c6*s4)-qdot2*c1*(l4*c3*c4*s2-l4*c2*s4-l7*c2*c6*s4+l7*c3*c4*c6*s2+l7*c2*c4*c5*s6+l7*c3*c5*s2*s4*s6)+l7*qdot5*s5*s6*(s1*s3*s4+c1*c4*s2-c1*c2*c3*s4),l7*qdot6*c6*(c3*c5*s1+c1*c2*c5*s3+c1*s2*s4*s5-c4*s1*s3*s5+c1*c2*c3*c4*s5)-l7*qdot5*s6*(c3*s1*s5+c1*c2*s3*s5-c1*c5*s2*s4+c4*c5*s1*s3-c1*c2*c3*c4*c5)-l7*qdot1*s6*(c2*c5*s1*s3-c1*c3*c5+c1*c4*s3*s5+s1*s2*s4*s5+c2*c3*c4*s1*s5)-l7*qdot3*s6*(c5*s1*s3-c1*c2*c3*c5+c3*c4*s1*s5+c1*c2*c4*s3*s5)+l7*qdot4*s5*s6*(s1*s3*s4+c1*c4*s2-c1*c2*c3*s4)-l7*qdot2*c1*s6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5),qdot6*(l7*(s5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)-c5*(c1*c3-c2*s1*s3))*(c6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)-s6*(c2*c4+c3*s2*s4))-l7*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3)*(s6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)+c6*(c5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+s5*(c1*c3-c2*s1*s3))))-qdot1*(l7*(s5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)-c5*(c3*s1+c1*c2*s3))*(s6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)+c6*(c2*c4+c3*s2*s4))+l7*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3)*(c6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)-s6*(c5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+s5*(c3*s1+c1*c2*s3))))+qdot3*(l7*c3*s1*s4*s6-l7*c6*s1*s3*s5+l7*c1*c2*c3*c6*s5+l7*c3*c4*c5*c6*s1+l7*c1*c2*s3*s4*s6+l7*c1*c2*c4*c5*c6*s3)-qdot4*(l7*c1*s2*s4*s6-l7*c4*s1*s3*s6+l7*c1*c2*c3*c4*s6+l7*c1*c4*c5*c6*s2+l7*c5*c6*s1*s3*s4-l7*c1*c2*c3*c5*c6*s4)+l7*qdot5*c6*(c3*c5*s1+c1*c2*c5*s3+c1*s2*s4*s5-c4*s1*s3*s5+c1*c2*c3*c4*s5)+l7*qdot2*c1*(c2*c4*s6-c2*c5*c6*s4+c3*s2*s4*s6-c6*s2*s3*s5+c3*c4*c5*c6*s2),l7*qdot6*(s6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)+c6*(c5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+s5*(c3*s1+c1*c2*s3)))-qdot2*(l7*(c1*s6*(s2*s3*s5+c2*c5*s4-c3*c4*c5*s2)+c1*c6*(c2*c4+c3*s2*s4))+l4*c1*(c2*c4+c3*s2*s4)+l2*c1*c2)-qdot1*(l4*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)+l7*(c6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)-s6*(c5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+s5*(c1*c3-c2*s1*s3)))-l2*s1*s2)-qdot3*(l7*(s6*(s5*(s1*s3-c1*c2*c3)-c4*c5*(c3*s1+c1*c2*s3))+c6*s4*(c3*s1+c1*c2*s3))+l4*s4*(c3*s1+c1*c2*s3))-qdot4*(l7*(c6*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+c5*s6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2))+l4*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4))-l7*qdot5*s6*(s5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)-c5*(c3*s1+c1*c2*s3)),qdot4*(l4*s1*(c2*s4-c3*c4*s2)+l7*s1*(c6*(c2*s4-c3*c4*s2)-c5*s6*(c2*c4+c3*s2*s4)))-qdot1*(l4*c1*(c2*c4+c3*s2*s4)+l2*c1*c2+l7*c1*(s6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)+c6*(c2*c4+c3*s2*s4)))+qdot2*(l4*s1*(c4*s2-c2*c3*s4)+l2*s1*s2+l7*s1*(s6*(c5*(s2*s4+c2*c3*c4)-c2*s3*s5)+c6*(c4*s2-c2*c3*s4)))-qdot3*(l7*s1*(s6*(c3*s2*s5+c4*c5*s2*s3)-c6*s2*s3*s4)-l4*s1*s2*s3*s4)-l7*qdot6*s1*(c6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)-s6*(c2*c4+c3*s2*s4))+l7*qdot5*s1*s6*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3),qdot4*(c1*c3-c2*s1*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-qdot3*(l4*c1*s3*s4+l4*c2*c3*s1*s4+l7*c1*c6*s3*s4-l7*c1*c3*s5*s6+l7*c2*c3*c6*s1*s4-l7*c1*c4*c5*s3*s6+l7*c2*s1*s3*s5*s6-l7*c2*c3*c4*c5*s1*s6)-qdot1*(l4*c3*s1*s4+l4*c1*c2*s3*s4+l7*c3*c6*s1*s4+l7*s1*s3*s5*s6+l7*c1*c2*c6*s3*s4-l7*c1*c2*c3*s5*s6-l7*c3*c4*c5*s1*s6-l7*c1*c2*c4*c5*s3*s6)+l7*qdot6*(c1*c6*s3*s5-c1*c3*s4*s6-c1*c3*c4*c5*c6+c2*c3*c6*s1*s5+c2*s1*s3*s4*s6+c2*c4*c5*c6*s1*s3)+qdot2*s1*s2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)+l7*qdot5*s6*(c1*c5*s3+c2*c3*c5*s1+c1*c3*c4*s5-c2*c4*s1*s3*s5),qdot1*(l4*c1*s2*s4-l4*c4*s1*s3+l4*c1*c2*c3*c4+l7*c1*c6*s2*s4-l7*c4*c6*s1*s3-l7*c1*c4*c5*s2*s6-l7*c5*s1*s3*s4*s6+l7*c1*c2*c3*c4*c6+l7*c1*c2*c3*c5*s4*s6)+qdot4*(l4*c4*s1*s2-l4*c1*s3*s4-l4*c2*c3*s1*s4+l7*c4*c6*s1*s2-l7*c1*c6*s3*s4-l7*c2*c3*c6*s1*s4+l7*c1*c4*c5*s3*s6+l7*c5*s1*s2*s4*s6+l7*c2*c3*c4*c5*s1*s6)+qdot3*(c1*c3-c2*s1*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-qdot2*s1*(l4*c3*c4*s2-l4*c2*s4-l7*c2*c6*s4+l7*c3*c4*c6*s2+l7*c2*c4*c5*s6+l7*c3*c5*s2*s4*s6)-l7*qdot6*(c1*c4*s3*s6+s1*s2*s4*s6+c2*c3*c4*s1*s6+c4*c5*c6*s1*s2-c1*c5*c6*s3*s4-c2*c3*c5*c6*s1*s4)-l7*qdot5*s5*s6*(c1*s3*s4-c4*s1*s2+c2*c3*s1*s4),l7*qdot1*s6*(c3*c5*s1+c1*c2*c5*s3+c1*s2*s4*s5-c4*s1*s3*s5+c1*c2*c3*c4*s5)+l7*qdot5*s6*(c1*c3*s5+c1*c4*c5*s3-c2*s1*s3*s5+c5*s1*s2*s4+c2*c3*c4*c5*s1)+l7*qdot6*c6*(c2*c5*s1*s3-c1*c3*c5+c1*c4*s3*s5+s1*s2*s4*s5+c2*c3*c4*s1*s5)+l7*qdot3*s6*(c1*c5*s3+c2*c3*c5*s1+c1*c3*c4*s5-c2*c4*s1*s3*s5)-l7*qdot4*s5*s6*(c1*s3*s4-c4*s1*s2+c2*c3*s1*s4)-l7*qdot2*s1*s6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5),l7*(qdot1*c1*c4*s2*s6-qdot5*c1*c3*c5*c6+qdot2*c2*c4*s1*s6+qdot1*c3*c6*s1*s5-qdot3*c1*c3*s4*s6+qdot3*c1*c6*s3*s5-qdot4*c1*c4*s3*s6+qdot6*c4*c6*s1*s2-qdot6*c1*c6*s3*s4+qdot6*c1*c3*s5*s6+qdot1*s1*s3*s4*s6-qdot4*s1*s2*s4*s6-qdot1*c1*c2*c3*s4*s6+qdot1*c1*c2*c6*s3*s5-qdot1*c1*c5*c6*s2*s4+qdot1*c4*c5*c6*s1*s3-qdot2*c2*c5*c6*s1*s4+qdot3*c2*c3*c6*s1*s5-qdot4*c2*c3*c4*s1*s6-qdot4*c4*c5*c6*s1*s2+qdot5*c2*c5*c6*s1*s3-qdot6*c2*c3*c6*s1*s4+qdot4*c1*c5*c6*s3*s4+qdot5*c1*c4*c6*s3*s5+qdot6*c1*c4*c5*s3*s6+qdot2*c3*s1*s2*s4*s6-qdot2*c6*s1*s2*s3*s5+qdot3*c2*s1*s3*s4*s6+qdot5*c6*s1*s2*s4*s5-qdot6*c2*s1*s3*s5*s6+qdot6*c5*s1*s2*s4*s6-qdot3*c1*c3*c4*c5*c6-qdot1*c1*c2*c3*c4*c5*c6+qdot2*c3*c4*c5*c6*s1*s2+qdot3*c2*c4*c5*c6*s1*s3+qdot4*c2*c3*c5*c6*s1*s4+qdot5*c2*c3*c4*c6*s1*s5+qdot6*c2*c3*c4*c5*s1*s6),0,qdot4*(l4*s2*s4+l4*c2*c3*c4+l7*c6*s2*s4+l7*c2*c3*c4*c6-l7*c4*c5*s2*s6+l7*c2*c3*c5*s4*s6)-qdot2*(l2*c2+l4*c2*c4+l7*c2*c4*c6+l4*c3*s2*s4+l7*c3*c6*s2*s4+l7*c2*c5*s4*s6+l7*s2*s3*s5*s6-l7*c3*c4*c5*s2*s6)-l7*qdot6*(c2*c3*s4*s6-c4*s2*s6-c2*c6*s3*s5+c5*c6*s2*s4+c2*c3*c4*c5*c6)-qdot3*c2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)+l7*qdot5*s6*(s2*s4*s5+c2*c5*s3+c2*c3*c4*s5),l7*qdot6*s2*(s3*s4*s6+c3*c6*s5+c4*c5*c6*s3)-qdot2*c2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)-qdot3*s2*(l4*c3*s4+l7*c3*c6*s4+l7*s3*s5*s6-l7*c3*c4*c5*s6)-qdot4*s2*s3*(l4*c4+l7*c4*c6+l7*c5*s4*s6)+l7*qdot5*s2*s6*(c3*c5-c4*s3*s5),qdot2*(l4*s2*s4+l4*c2*c3*c4+l7*c6*s2*s4+l7*c2*c3*c4*c6-l7*c4*c5*s2*s6+l7*c2*c3*c5*s4*s6)-qdot4*(l4*c2*c4+l7*c2*c4*c6+l4*c3*s2*s4+l7*c3*c6*s2*s4+l7*c2*c5*s4*s6-l7*c3*c4*c5*s2*s6)+l7*qdot6*(c2*s4*s6+c2*c4*c5*c6-c3*c4*s2*s6+c3*c5*c6*s2*s4)-qdot3*s2*s3*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-l7*qdot5*s5*s6*(c2*c4+c3*s2*s4),l7*qdot2*s6*(s2*s4*s5+c2*c5*s3+c2*c3*c4*s5)-l7*qdot5*s6*(s2*s3*s5+c2*c5*s4-c3*c4*c5*s2)+l7*qdot6*c6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5)-l7*qdot4*s5*s6*(c2*c4+c3*s2*s4)+l7*qdot3*s2*s6*(c3*c5-c4*s3*s5),l7*qdot4*(c2*s4*s6+c2*c4*c5*c6-c3*c4*s2*s6+c3*c5*c6*s2*s4)-l7*qdot6*(c2*c4*c6+c3*c6*s2*s4+c2*c5*s4*s6+s2*s3*s5*s6-c3*c4*c5*s2*s6)-l7*qdot2*(c2*c3*s4*s6-c4*s2*s6-c2*c6*s3*s5+c5*c6*s2*s4+c2*c3*c4*c5*c6)+l7*qdot5*c6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5)+l7*qdot3*s2*(s3*s4*s6+c3*c6*s5+c4*c5*c6*s3);

    return dJ;

}

// the tolerance is e ( change if needed)
//damped pesudo inverse

//template <typename Derived1, typename Derived2>
void dampedPseudoInverse(const Eigen::MatrixXd& A,double dampingFactor,double e,Eigen::MatrixXd& Apinv,unsigned int computationOptions)
{
    int m = A.rows(), n = A.cols(), k = (m < n) ? m : n;
    //JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(computationOptions);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd = A.jacobiSvd(computationOptions);
    //const typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType& singularValues = svd.singularValues();
    const typename Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType& singularValues = svd.singularValues();
    Eigen::MatrixXd sigmaDamped = Eigen::MatrixXd::Zero(k, k);
    double damp = dampingFactor * dampingFactor;

    for (int idx = 0; idx < k; idx++)
    {
        if(singularValues(idx)>=e)damp=0.0;
        else damp=(1-((singularValues(idx)/e)*(singularValues(idx)/e)))*dampingFactor * dampingFactor;

        sigmaDamped(idx, idx) = singularValues(idx) / ((singularValues(idx) * singularValues(idx)) + damp);
    }
    Apinv = svd.matrixV() * sigmaDamped * svd.matrixU().transpose(); // damped pseudoinverse
}


//dampingFactor is 0.1( change if needed)
//return pesudo inverse computed in dampedPseudoInverse function
/*
Eigen::MatrixXd compute_Damped_pinv(Eigen::MatrixXd j, double f, double e)
{

    int row,col;
    row=j.rows();
    col=j.cols();
    Eigen::MatrixXd pJacobian(col,row);

    dampedPseudoInverse(j,f,e,pJacobian,Eigen::ComputeThinU|Eigen::ComputeThinV );

    return pJacobian;
}
*/
/*
//weighted pseudo inverse
MatrixXd compute_Weighted_Damped_pinv(MatrixXd j,MatrixXd Q)
{
    int row,col;
    row=j.rows();
    col=j.cols();

    MatrixXd J_inv_w(col,row); MatrixXd J_a(row,col);

    Q=Q.array().sqrt();
    Q=compute_Damped_pinv(Q,0.0,0.0);
    J_a=j*(Q);

    J_inv_w=(Q)*compute_Damped_pinv(J_a,0.1,0.1);

    return J_inv_w;
}
*/

//IN ORDER TO HAVE METHOD FOR FILTERING ALSO IN LEARNING CLASS
Kuka_Vec Filter2(std::vector<Kuka_Vec> &signal, int filter_length)
{
    int signal_length = signal.size();
    
    Kuka_Vec output = Kuka_Vec::Constant(0.0);

    if(signal_length > filter_length)
    {
            for(int i=0;i<filter_length;i++)
            {
                output = output + signal[signal_length - i -1];
            }
            output = output / filter_length;
    }
    else
    {
        output = signal.back();
    }
    return output;
};

//IN ORDER TO HAVE METHOD FOR FILTERING ALSO IN LEARNING CLASS FOR DATASETS
Eigen::VectorXd Filter3(std::vector<Eigen::VectorXd> &signal, int filter_length)
{
    int signal_length = signal.size();
    
    Eigen::VectorXd output = Eigen::VectorXd::Constant(DIMY,0.0);

    if(signal_length > filter_length)
    {
            for(int i=0;i<filter_length;i++)
            {
                output = output + signal[signal_length - i -1];
            }
            output = output / filter_length;
    }
    else
    {
        output = signal.back();
    }
    return output;
};
