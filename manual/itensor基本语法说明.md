# one site 单节点

>Site
>
>意指"节点".

单节点的自旋为 $\frac{1}{2}$ 的波函数,有两个基底.

$|s=1\rangle=|\uparrow\rangle;|s=2\rangle=|\downarrow\rangle$

对于一般的自旋为 $\frac{1}{2}$ 的波函数,可以写成如下形式:

$|\psi\rangle=\sum_{s=1}^{2}\psi_s|s\rangle(\psi_s\in\mathcal{C})$

使用张量数学来处理这个单节点波函数,
即 $|\psi_s\rangle$ 对应一个有一个"触手"的结点.展开形式为 $(^{\psi_1}_{\psi_2})$ , $\psi_1,\psi_2$ 又各自是一个单"触手"的结点.

```cpp
Index s("s",2);
//"s"代表的是Index的命名
//2是Index的维度
ITensor psi(s);
//由于只声明不定义,所以初始值为0
```
>**Index 指标**
>
>指的是表示某种物理量的符号.这个符号
可以是矢量, 张量, 算符.
>
>```cpp
>Index s(2,"s");
>Index t(3,"t");
>ITensor T(s,t);
>```
>这个表示的就是创建了一个二维的物理量,我们用$s$来标记它;物理量t则是三维的.
而Itensor的大小则是 $2\times 3$.

对 $|\psi_s\rangle$ 进行初始化操作.我们不妨将其初始化为基矢:

$|\psi_s\rangle=|\uparrow\rangle$

```cpp
Index s(2,"s");
//在最新的ITensor语法规范中,Index (string, int)的语法已经被弃用
//目前的语法应当是Index (int, string)
ITensor psi(s);
psi(s(1)) = 1;
PrintData(psi);
```

本示例代码应该会输出:

```bash
psi = 
ITensor ord=1: (dim=2|id=414|"s") 
{norm=1.00 (Dense Real)}
(1) 1.0000000
```

## Operators 操作符/算子 $\hat{A}$

算子体现在张量网络上就是一个拥有两个"触手"的结点, 它可以和只有一个"触手"的结点结合, 从而生成一个新的只有一个触手的结点.

```cpp
ITensor Sz(s,prime(s));
ITensor Sx(s,prime(s));
//prime(s)是指的对s进行共轭操作
```

如果要对一个向量或者矩阵进行转置,共轭等操作,我们可以用被称为**上下标**的操作来表示.比如, $A'_{ij}=A_{ji}$, $A^{\dagger}_{ij}=A_{ji}$ 等.

```prime(s)```就代表了对指标进行上标操作所得到的量.

>**Prime Level** 上标级别
>
>指标有一个属性被称为**prime level**, 表示是否被某个操作升降级.默认情况下指标的上标级别为0.
>
>这个量的作用是用来对两个张量进行比较, 在进行某些操作时, 上标必须要满足某些需求运算才能够成立.
>
>比如
>```cpp
>Index s(2,"s");
>Index t(2,"2");
>ITensor A(s,t);
>ITensor B(s,t);// s, t 都是上标级别为 0 的指标
>C = A * prime(B);// prime(B) 表示将 B 中的所有指标上标级别加 1
>```
>在这里, ```A * prime(B)```就是一个不允许指标重复的操作.所以为了完成这个计算,我们需要对```B```进行上标级别升级的操作,也就是使用```prime()```函数.

现在我们来对 $\hat{S}_z,\hat{S}_x$ 算子进行定义.
```cpp
ITensor Sz(s,prime(s)),Sx(s,prime(s));

// commaInit(Sz,s,prime(s)) =  0.5, 0.0,
//                             0.0,-0.5;
// commaInit(Sx,s,prime(s)) =  0.0, 0.5,
//                             0.5, 0.0;
//commaInit的语法已经被弃用, 应当使用.set()
//Sz
Sz.set(s(1),prime(s)(1),0.5);
Sz.set(s(2),prime(s)(2),-0.5);
//Sx
Sx.set(s(1),prime(s)(2),0.5);
Sx.set(s(2),prime(s)(1),0.5);//设置分量
```
现在我们尝试对波函数 $|\psi\rangle$ 进行求某方向自旋的操作.

即方程上的

$(\hat{S}_x)_{s'}^{s}\psi_{s}$

对应的是
```cpp
ITensor phi = Sx * psi;
```

(注意算符的上标要和波函数的下标相同, 这样的操作才是被允许的. $s$和$s'$的上标等级不同, 所以是不匹配的)

为了研究经过求自旋的量 $|\phi\rangle$ 的具体情况, 我们将其输出在终端:

```cpp
ITensor phi = Sx * psi;
PrintData(phi);
```
输出的结果应该是:
```bash
phi = 
ITensor ord=1: (dim=2|id=816|"s")' 
{norm=0.50 (Dense Real)}
(2) 0.5000000
```
我们来看看更多的结果. 如果有 $|\psi_s\rangle=(|^{\cos{\frac{\theta}{2}}}_{\sin{\frac{\theta}{2}}}\rangle)_{\theta=\frac{\pi}{4}}$, 那么我们可以这样描述:

```cpp
Real theta = Pi/4;
// psi(s(1)) = cos(theta/2);
// psi(s(2)) = sin(theta/2);
//该语法已被弃用,使用.set()替代
psi.set(s(1),cos(theta/2));
psi.set(s(2),sin(theta/2));
PrintData(psi);
```
终端会输出
```bash
psi = 
ITensor ord=1: (dim=2|id=977|"s") 
{norm=1.00 (Dense Real)}
(1) 0.9238795
(2) 0.3826834
```
## 期望值 $\langle\hat{A}\rangle$
在物理上我们定义算符 $\hat{A}$ 的期望值为 $\langle\psi|\hat{A}|\psi\rangle$. 

我们不妨求自旋算符 $\hat{S_z}$ 的期望值. 在张量网络上, 这个过程体现为, 作为算符的"具有上下两个触手的结点"的每个触手分别连接一个"只有单触手的结点"(也就是代表着单节点波函数).

我们用程序语言来描述这个过程:
```cpp
ITensor cpsi = dag(prime(psi));
// Real zz = (cpsi * Sz * psi).toReal();
// Real xx = (cpsi * Sx * psi).toReal();
// toReal()函数已弃用,使用.real()替代
Real zz = (cpsi * Sz * psi).real();
Real xx = (cpsi * Sx * psi).real();
// println("<Sz>=",zz);
// println("<Sx>=",xx);
// println()已弃用,使用printfln()替代
printfln("<Sz>=",zz);
printfln("<Sx>=",xx);
```
其中```ITensor cpsi = dag(prime(psi));```就是一个将 $\psi$ 先取转置后取共轭的过程, 所以```cpsi```的上标级数为$2$. 我们也可以推测得知, 算符 $\hat{S}_x,\hat{S}_z$的上标级数为$1$.

终端输出为

```bash
<Sz>=0.353553
<Sx>=0.353553
```

观察到 $\sqrt{\langle S_z\rangle ^2 + \langle S_x\rangle ^2} = \frac{1}{2}$ 

我们可以将这个过程进行类似结合率的分析:

$\langle\psi|\hat{S}_z|\psi\rangle = \langle\psi|\hat{S}_z\psi\rangle$.

用代码来描述结合律的过程:
```cpp
ITensor Zpsi = Sz * psi;
//因为Sz以s为上标,psi以s为下标,所以允许运算
ITensor expect = cpsi * Zpsi;
Real zz = expect.real()
```

## Quiz $1$ 解析

>提示
>
>原文所提示的函数```elt()```已经弃用.所以采用的是传统的```prime(A) * B```方式. 如果你想使用函数而非 ```*```来得到内积, 你应当采用函数```inner()```

```cpp
#include "itensor/all.h"
#include "itensor/util/print_macro.h"

using namespace itensor;

int main(){
    // Define our Index 
    // (the space we are working with)
    auto s = Index(2,"s");

    // Operators 
    auto Sx = ITensor(s,prime(s));
    Sx.set(s(1),prime(s)(2),0.5);
    Sx.set(s(2),prime(s)(1),0.5);
    PrintData(Sx);

    // Single-site wavefunction
    auto psi = ITensor(s); //initialized to zero

    // TODO 
    // 1. make the above wavefunction
    //    the (normalized) positive Sx eigenstate

    psi.set(s(1),1/sqrt(2));
    psi.set(s(2),1/sqrt(2));
    PrintData(psi);

    // TODO
    // 2. Compute |phi> = Sx |psi> using
    //    the Sx and psi ITensors above
    //    AND
    //    compute: auto olap = <psi|phi>
    //    using the * operator and elt(...) method.
    //    Print the result with PrintData(...).
    auto phi = Sx * psi;
    auto olap = prime(psi) * phi;
    //auto olap = elt(prime(psi),phi);
    PrintData(olap);

    // TODO
    // 3. Try normalizing |phi> and recompute
    //    the inner product <psi|phi>
    //    Print the result with PrintData(...).
    //    HINT: use phi /= norm(phi)) to normalize.

    phi /= norm(phi);
    //auto olap2 = eltC(prime(psi),phi);
    auto olap2 = prime(psi) * phi;
    PrintData(olap2);

    return 0;
}
```
输出得到:

```bash
Sx = 
ITensor ord=2: (dim=2|id=15|"s") (dim=2|id=15|"s")' 
{norm=0.71 (Dense Real)}
(2,1) 0.5000000
(1,2) 0.5000000

psi = 
ITensor ord=1: (dim=2|id=15|"s") 
{norm=1.00 (Dense Real)}
(1) 0.7071068
(2) 0.7071068

olap = 
ITensor ord=0: 
{norm=0.50 (Dense Real)}
  0.5000000

olap2 = 
ITensor ord=0: 
{norm=1.00 (Dense Real)}
  1.0000000
```

# Two Sites 双节点

## 纠缠态

对于双自旋的波函数, 我们一般的描述方程是:

$|\Psi\rangle = \sum_{s_1,s_2 = 1}^{2}\psi_{s_1s_2}|s_1\rangle|s_2\rangle$

体现在张量网络上, 就是一个结点在同一方向上同时有着两个"触手". 
对于我们写出来的态,我们可以分离出两个单态(Singlet).

程序上的描述方法:
```cpp
// Index s1(2,"s1",Site), s2(2,"s2",Site);
// 该语法已经被弃用,使用两步语法来替代
Index s1(2,"s1");s1.addTags("Site");
Index s2(2,"s2");s1.addTags("Site");
ITensor psi(s1,s2);                 //只声明不定义,则默认为0
// psi(s1(1),s2(2)) = 1./sqrt(2);
// psi(s1(2),s2(1)) =-1./sqrt(2);
// 该语法已被弃用,使用.set()来替代.
psi.set(s1(1),s2(2), 1./sqrt(2));
psi.set(s1(2),s2(1),-1./sqrt(2));
PrintData(psi);
```
>```Index s1(2,"s1");s1.addTags("Site");```代表的含义是创建一个维数为```int```,标签(或者理解为"名字",一般是指物理上的状态,比如自旋,粒子位置等等)为```string```,指标类型为```Site```的物理量.同时使用```ITensor psi(s1,s2)```将```s1```和```s2```设置为纠缠态(**Entangled State**).

现在要说明的就是指标(```Index```)的类型:```Site```和```Link```.

```Site```表示单个量子位的物理态. 适用于表示单个自旋,单个粒子的位置等等. 与之配对的维数```int```表示的是该量子位可以取的物理态的数量;

```Link```表示的是不同量子位之间的相互作用. 这种相互作用通常表示不同量子位的链接, 适用于表示各种相互作用, 比如哈密顿量等.

~~(还有表示不同于```Site```和```Link```类型的其它指标类型, 比如```Bulk```)~~

上面的代码中我们用```Site```来标记两个量子位,并且使其结合为纠缠态. 

结果输出为

```bash
psi = 
ITensor ord=2: (dim=2|id=324|"s1,Site") (dim=2|id=127|"s2") 
{norm=1.00 (Dense Real)}
(2,1) -0.707107
(1,2) 0.7071068
```

## 哈密顿量$\hat{H}$

我们写出双自旋系统的哈密顿量方程:

$\hat{H}=\hat{S}_1\cdot\hat{S}_2=S_1^zS_2^z+\frac{1}{2}S_1^+S_2^-+\frac{1}{2}S_1^-S_2^+$

其中 $S_1^\pm, S_2^\pm$ 是升降算符. 我们在程序中这样创建:

```cpp
Index s1(2,"s1");s1.addTags("Site");
Index s2(2,"s2");s1.addTags("Site");

ITensor Sz1(s1,prime(s1)),Sp1(s1,prime(s1)),Sm1(s1,prime(s1));
ITensor Sz2(s2,prime(s2)),Sp2(s2,prime(s2)),Sm2(s2,prime(s2));

// commaInit(Sp1,s1,prime(s1)) =   0, 1,
//                                 0, 0;
// commaInit语法已被弃用.

//Sz1,Sz2
Sz1.set(s1(1),prime(s1)(1),0.5);Sz1.set(s1(2),prime(s1)(2),-0.5);
Sz2.set(s2(1),prime(s2)(1),0.5);Sz2.set(s2(2),prime(s2)(2),-0.5);

//Sp1,Sp2
Sp1.set(s1(1),prime(s1)(2),1);Sp2.set(s2(1),prime(s2)(2),1);

//Sm1,Sm2
Sm1.set(s1(2),prime(s1)(1),1);Sm2.set(s2(2),prime(s2)(1),1);

ITensor H = Sz1 * Sz2 + 0.5 * Sp1 * Sm2 + 0.5 * Sm1 * Sp2;
PrintData(H);//你也可以尝试执行printfln("H=",H);
```

执行后可以得到结果
```bash
H = 
ITensor ord=4: (dim=2|id=397|"s1,Site") (dim=2|id=397|"s1,Site")' (dim=2|id=53|"s2") (dim=2|id=53|"s2")' 
{norm=0.87 (Dense Real)}
(1,1,1,1) 0.2500000
(2,2,1,1) -0.250000
(1,2,2,1) 0.5000000
(2,1,1,2) 0.5000000
(1,1,2,2) -0.250000
(2,2,2,2) 0.2500000
```

>这表明这是一个四阶张量,并且由索引```s1```, ```s1'```, ```s2```, ```s2'```来描述. 它们各自的维度都是$2$, 即它们可以取值为$1$或$2$.这就是```(i,j,k,l)```中```1```或者```2```的含义, 后面的值是对应的张量元素.

这样计算得来的 $\hat{H}$ 在张量网络中表示一个上下各自有两个"触手"的结点, 恰好可以和表现为在某一方向上拥有两个"触手"的纠缠态结点进行结合.

更具体地说, 我们上面创建的 $\hat{H}$ 的上标是```s1'```和```s2'```,下标是```s1```和```s2```;
而 $\psi$ 的上标是```s1```和```s2```,两者进行结合就可以得到只有两个上标```s1'```和```s2'```的新节点 $\hat{H}\psi$ .

程序上这样描述这个结合的过程:

```cpp
ITensor Hpsi = H * psi;
//Hpsi.mapprime(1,0);//该语句的作用是将Hpsi中所有上标级别为1的索引全部降为0,使得之后的运算能够被允许
//该语法已被弃用. 使用.noPrime()来代替
Hpsi.noPrime();
Real E = (dag(psi) * Hpsi).real();
//Print()语法已被弃用, 使用PrintData()来代替.
PrintData(E);
```
能够在终端得到输出
```bash
E = -0.75
```
