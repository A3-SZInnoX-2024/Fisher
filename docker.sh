#!/bin/bash

# 打印信息函数
print_info() {
    echo -e "\e[32m$1\e[0m"
}

# 检查系统架构
osarch=$(dpkg --print-architecture)

# 安装必要的软件包
print_info "正在安装必要的软件包..."
sudo apt install apt-transport-https ca-certificates curl software-properties-common -y

# 添加 Docker 的 GPG 密钥
print_info "添加 Docker 的 GPG 密钥..."
curl -fsSL https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu/gpg | sudo apt-key add -

# 验证密钥
sudo apt-key fingerprint 0EBFCD88

# 根据系统架构添加源
if [ "$osarch" == "amd64" ]; then
    print_info "为 amd64 架构添加源..."
    sudo add-apt-repository "deb [arch=amd64] https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu $(lsb_release -cs) stable" -y
elif [ "$osarch" == "arm64" ]; then
    print_info "为 arm64 架构添加源..."
    sudo add-apt-repository "deb [arch=arm64] https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu $(lsb_release -cs) stable" -y
else
    echo "不支持的架构: $osarch"
    exit 1
fi

# 更新源并安装 Docker CE
print_info "更新源并安装 Docker CE..."
sudo apt update
sudo apt --fix-broken install -y
sudo apt install docker-ce -y

# 添加用户到 docker 组
sudo groupadd docker
user=$(id -un)
sudo gpasswd -a $user docker

print_info "安装完成！你可以使用 'docker --version' 指令测试是否正常。"

# Docker 登录信息
DOCKER_USERNAME="wzx1210"
DOCKER_PASSWORD="innox2023"
DOCKER_REGISTRY="registry.cn-hangzhou.aliyuncs.com"
DOCKER_IMAGE="registry.cn-hangzhou.aliyuncs.com/wzx1210/innox2024_wc:v3.0.3"

print_info "正在登录 Docker..."
echo $DOCKER_PASSWORD | sudo docker login --username $DOCKER_USERNAME --password-stdin $DOCKER_REGISTRY
sudo docker pull $DOCKER_IMAGE
