在某文件夹下创建ITensor文件夹。在该文件夹内执行

```bash
git clone https://github.com/ITensor/ITensor itensor
```
得到以itensor命名的编译源文件。这个文件夹名字可以自己改。

# 前置：安装lapack和blas依赖库.

```bash
sudo apt-get update
sudo apt-get install libblas-dev liblapack-dev
```

检验是否正确安装:
    
```bash
dpkg -L libblas-dev
dpkg -L liblapack-dev
```
若能弹出对应路径说明安装成功。

---

在终端执行

```bash
cd itensor
```

进入下载的库文件所在的文件夹.

```bash
cp options.mk.sample options.mk
```

将示例mk文件另存为mk文件.并且执行

```bash
sudo nano options.mk
```

以nano编辑器打开配置文件。配置文件里默认的平台是MacOS，我们用的是WSL2,所以将

```mk
PLATFORM=macos
BLAS_LAPACK_LIBFLAGS=-framework Accelerate
```

这两行注释掉，而将下面的

```mk
#PLATFORM=lapack
#BLAS_LAPACK_LIBFLAGS=-lpthread -L/usr/lib -lblas -llapack
```

取消注释。ctrl+O，ENTER保存。ctrl+X退出。

依然是在itensor目录下，在bash终端执行

```bash
make
```

等待终端执行完成。

为了能够在ITensor源文件所在文件夹以外的路径使用Itensor,
应该遵循以下步骤。

1. 将itensor/tutorial/project_template下的所有文件(Makefile,myappname.cc,myclass.h,当然说明书README.md无关紧要)复制粘贴到含有main函数代码的cpp文件所在的文件夹中;

2. 修改LIBRARY_DIR.这个路径和options.mk所在的文件夹的路径是相同的.(如果有/mnt/e/wsl/ubuntu2204/cpp/itensorCalculation/ITensor/itensor/options.mk,那么就将

```Makefile
LIBRARY_DIR=$(HOME)/itensor
```

改写为

```Makefile
LIBRARY_DIR=/mnt/e/wsl/ubuntu2204/cpp/itensorCalculation/ITensor/itensor
```

即可;

3. 修改APPNAME.这个是你的程序的名字.我给自己的程序源文件主程序起名为ctnmodel.cc,
所以

```Makefile
APP=myappname
```

改为

```Makefile
APP=ctnmodel
```

4. 增加依赖库.模板里的myclass.h是用来测试的,在实际过程中可以替换为自己的头文件;

5. 编译的源文件名字,只要完成了APP的修改就无需改动.

然后在bash终端执行

```bash
make
```

编译出执行文件.

>**注意**
>
>示例的Makefile并没有添加可执行权限，如果你想在bash终端中进行执行，那么你可以
>
>```bash
>chmod +x ctnmodel
>```
>(ctnmodel换成你自己程序的名字)
>
>或者直接在你的Makefile里面找到build和debug的行，
>然后分别添加
>```
>chmod +x $(APP)
>```
>
>和
>```
>chmod +x $(APP)-g
>```
>最后形成这样的表达：
>
>```
>build: $(APP)
>#这一行是为了添加可执行权限，如果你不需要，可以将其注释掉
>	chmod +x $(APP)
>debug: $(APP)-g
>#同上
>	chmod +x $(APP)-g
>```

你可以通过在终端执行

```bash
./ctnmodel
```

(ctnmodel换成你的程序的名字)来完成程序的执行.这些模板都是由Makefile来进行管理的,所以对于vscode的编译执行功能没有得到充分发挥.


待续:vscode的配置.

<!-- 一个解决的办法是在VSCode里配置includePath.

你可以在工程文件夹里新建一个c_pp_properties.json并且写入以下内容:

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**/*.h",
                "${workspaceFolder}/**/*.hpp",
                "/mnt/e/wsl/ubuntu2204/cpp/itensorCalculation/ITensor/itensor/*"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/g++",
            "cStandard": "gnu17",
            "cppStandard": "gnu++14",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}

```

新建一个tasks.json并且写入以下内容:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build",
            "type": "shell",
            "command": "make build",
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": "$gcc"
        },
        {
            "label": "debug",
            "type": "shell",
            "command": "make debug",
            "group": "build",
            "problemMatcher": "$gcc"
        }
    ]
}
```

新建launch.json文件并且写入:
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/ctnmodel",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": true,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
``` -->
