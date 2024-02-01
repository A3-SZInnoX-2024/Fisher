#!/bin/bash

if [ "$(id -u)" != "0" ]; then
   echo "该脚本必须以root权限运行" 1>&2
   exit 1
fi

# 更换系统源至清华大学源
echo "更换系统源..."
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup  # 备份原有源
echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse" | sudo tee /etc/apt/sources.list
echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-security main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list

# 更新系统包列表
echo "更新系统包列表..."
sudo apt update

echo "安装Python 3.8..."
sudo apt install -y python3.8

echo "创建软链接..."
sudo ln -sf /usr/bin/python3.8 /usr/bin/python3

echo "验证Python安装..."
python3 --version

# 更换pip源至清华大学源
echo "更换pip源至清华大学源..."
mkdir -p $HOME/.pip
echo "[global]" > $HOME/.pip/pip.conf
echo "index-url = https://pypi.tuna.tsinghua.edu.cn/simple" >> $HOME/.pip/pip.conf

# 输出pip配置信息验证
echo "验证pip配置..."
cat $HOME/.pip/pip.conf

# 卸载已安装的docker相关包
echo "卸载已存在的Docker相关包..."
for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do 
    sudo apt-get remove -y $pkg
done

# 安装必要的证书和工具
echo "安装必要的证书和工具..."
sudo apt-get install -y ca-certificates curl gnupg

# 添加官方GPG key
echo "添加Docker官方GPG密钥..."
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 设置下载源地址
echo "设置Docker下载源地址..."
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 更新apt
echo "更新apt..."
sudo apt-get update

# 下载docker相关软件包
echo "安装Docker及其相关组件..."
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 创建docker用户组并将当前用户加入
echo "添加当前用户到docker用户组..."
sudo groupadd docker
sudo usermod -aG docker $USER

# 测试用户组设置
echo "测试docker用户组设置..."
newgrp docker
docker run hello-world

# 创建并编辑/etc/docker/daemon.json文件
echo "设置Docker镜像源..."
sudo touch /etc/docker/daemon.json
echo '{
 "registry-mirrors": ["https://registry.docker-cn.com"]
}' | sudo tee /etc/docker/daemon.json

# 重启Docker服务
echo "重启Docker服务..."
sudo systemctl daemon-reload
sudo systemctl restart docker

# 重启电脑提示
echo "请重启您的电脑以应用更改."

# 验证Docker安装
echo "测试Docker安装..."
sudo docker run hello-world

# 验证设置
echo "验证Docker设置..."
docker info | grep "Registry Mirrors"
echo "脚本执行完毕."
