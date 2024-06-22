# 使用 Python 3.11 镜像作为基础镜像
FROM python:3.11

# 安装 Vim
RUN apt-get update && apt-get install -y vim

# 安装 OpenJDK 11
# RUN apt-get install -y openjdk-11-jdk

# 设置 JAVA_HOME 环境变量
# ENV JAVA_HOME /usr/lib/jvm/java-11-openjdk-amd64

# 安装 antlr4-tools
RUN pip install antlr4-tools antlr4-python3-runtime

RUN echo "y" | antlr4

# 设置 antlr4 可执行文件的路径
# ENV PATH "$PATH:/usr/local/lib/python3.10/site-packages/antlr4/bin"

# 在 Docker 镜像中使用 ANTLR
# 示例：在 Dockerfile 中添加你的 ANTLR 代码生成步骤
# 如需执行 antlr4 命令生成 Python 程序，可以在 Dockerfile 中添加相应的命令

