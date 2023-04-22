#include "itensor/all.h"
// #include "myclass.h"

using namespace itensor;

int main(){
    //Construct and print a MyClass object

    //The MyClass files are included just
    //to demonstrate how to create extra code
    //in separate files to add to your project
    // auto m = MyClass("m",5);
    // println(m);

    // Construct and print an ITensor

    // auto a = Index(5,"A");auto b = Index(4,"B");

    // auto T = ITensor(a,b);T.randomize();

    // //The PrintData macro prints "T ="
    // //followed by information about T 
    // //and all of its non-zero elements
    // PrintData(T);
    // return 0;
    // Index s(2,"s");
    // ITensor psi(s);
    // psi.set(s(1),1);
    // PrintData(psi);

    // ITensor Sz(s,prime(s)),Sx(s,prime(s));
    // //Sz
    // Sz.set(s(1),prime(s)(1),0.5);
    // Sz.set(s(2),prime(s)(2),-0.5);
    // //Sx
    // Sx.set(s(1),prime(s)(2),0.5);
    // Sx.set(s(2),prime(s)(1),0.5);

    // ITensor phi = Sx * psi;
    // PrintData(phi);

    // Real theta = Pi/4;
    // //psi(s(1)) = cos(theta/2);
    // psi.set(s(1),cos(theta/2));
    // psi.set(s(2),sin(theta/2));
    // PrintData(psi);

    // ITensor cpsi = dag(prime(psi));
    // Real zz = (cpsi * Sz * psi).real();
    // Real xx = (cpsi * Sx * psi).real();
    // printfln("<Sz>=",zz);
    // printfln("<Sx>=",xx);

    Index s1(2,"s1");s1.addTags("Site");
    Index s2(2,"s2");s1.addTags("Site");
    ITensor psi(s1,s2);//只声明不定义,则默认为0
    psi.set(s1(1),s2(2), 1./sqrt(2));
    psi.set(s1(2),s2(1),-1./sqrt(2));
    // PrintData(psi);

    ITensor Sz1(s1,prime(s1)),Sp1(s1,prime(s1)),Sm1(s1,prime(s1));
    ITensor Sz2(s2,prime(s2)),Sp2(s2,prime(s2)),Sm2(s2,prime(s2));
    // commaInit(Sp1,s1,prime(s1)) =   0, 1,
    //                                 0, 0;
    // commaInit语法已被弃用.

    //Sz1,Sz2
    Sz1.set(s1(1),prime(s1)(1),0.5);
    Sz1.set(s1(2),prime(s1)(2),-0.5);
    Sz2.set(s2(1),prime(s2)(1),0.5);
    Sz2.set(s2(2),prime(s2)(2),-0.5);

    //Sp1,Sp2
    Sp1.set(s1(1),prime(s1)(2),1);
    Sp2.set(s2(1),prime(s2)(2),1);

    //Sm1,Sm2
    Sm1.set(s1(2),prime(s1)(1),1);
    Sm2.set(s2(2),prime(s2)(1),1);

    ITensor H = Sz1 * Sz2 + 0.5 * Sp1 * Sm2 + 0.5 * Sm1 * Sp2;
    //printfln("H=",H);
    //PrintData(H);

    ITensor Hpsi = H * psi;
    //Hpsi.mapprime(1,0);//该语句的作用是将Hpsi中所有上标级别为1的索引全部降为0,使得之后的运算能够被允许
    Hpsi.noPrime();
    Real E = (dag(psi) * Hpsi).real();
    PrintData(E);
}