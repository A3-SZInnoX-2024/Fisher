# 使用 VScode 连接到香橙派

## 安装 Remote-SSH 插件

1. 下载插件 Remote-SSH , 点击左侧的插件图表，搜索 Remote-SSH，点击安装。

    ![vscode_remoteSSH](/docs/pitures/vscode_remoteSSH.jpg)

2. 安装成功后，Vscode 左侧将出现一个图标，点击打开。

    ![vscode_setRemoteSSH01](/docs/pitures/vscode_setRemoteSSH01.jpg)

## 使用 VScode 连接到香橙派

3. 打开后将鼠标移动到 SSH 这一行，点击 右侧的加号，新建远程

    ![vscode_setRemoteSSH02](/docs/pitures/vscode_setRemoteSSH02.jpg)

4. 在上方会出现这样的一个界面，输入`ssh orangepi@<ip>`, 将`<ip>`换成你的 orangepi 的 ip 。

    (例如`ssh orangepi@10.10.42.231`)，然后回车

    ![vscode_setRemoteSSH03](/docs/pitures/vscode_setRemoteSSH03.jpg)

5. 选择第一个选项

    ![vscode_setRemoteSSH04](/docs/pitures/vscode_setRemoteSSH04.jpg)

6. 在左侧的界面点击刷新一下

    ![vscode_setRemoteSSH05](/docs/pitures/vscode_setRemoteSSH05.jpg)

7. 可以看到你的 orangepi 已经出现在这里了，点击连接，出现一个输入密码的窗口，输入密码(密码是`orangepi`)，回车

    ![vscode_setRemoteSSH06](/docs/pitures/vscode_setRemoteSSH06.jpg)

8. 右下角会显示正在下载 Vscode 服务器，请稍作等待，如果一直卡住不动回到第 1 步重新安装 Remote-SSH , 可以选择换一个版本安装

9. 当左下角出现这个图标的时候，说明连接成功了，ip 是你的 orangepi ip

    ![vscode_setRemoteSSH07](/docs/pitures/vscode_setRemoteSSH07.jpg)