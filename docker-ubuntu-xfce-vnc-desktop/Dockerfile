FROM tensorflow/tensorflow:2.1.0rc2-gpu-py3-jupyter
MAINTAINER Chieh Yu <welkineins@gmail.com>

ENV HOME /root
ENV DEBIAN_FRONTEND noninteractive
ENV LANG zh_CN.UTF-8

RUN apt-get update \
	&& apt-get install -y supervisor \
		openssh-server vim-tiny \
		xfce4 xfce4-goodies \
		x11vnc xvfb \
		firefox \
		language-pack-zh-hans \
		fonts-droid-fallback ttf-wqy-zenhei ttf-wqy-microhei fonts-arphic-ukai fonts-arphic-uming \
	&& apt-get autoclean \
	&& apt-get autoremove \
	&& rm -rf /var/lib/apt/lists/*

WORKDIR /root

ADD startup.sh ./
ADD supervisord.conf ./

RUN mkdir /var/run/sshd
RUN mkdir /root/.pip
RUN echo 'root:123' |chpasswd
RUN sh -c "echo 'PermitRootLogin yes' >> /etc/ssh/sshd_config"
RUN sh -c "echo 'PasswordAuthentication yes' >> /etc/ssh/sshd_config"

RUN sh -c "echo 'export LANG="zh_CN.UTF-8"' >> /etc/bash.bashrc"
RUN sh -c "echo 'export LC_ALL="zh_CN.UTF-8"' >> /etc/bash.bashrc"
RUN sh -c "locale-gen zh_CN.UTF-8"

RUN sh -c "echo '[global] \n index-url = https://pypi.tuna.tsinghua.edu.cn/simple' >> /root/.pip/pip.conf"

EXPOSE 5900
EXPOSE 22

ENTRYPOINT ["./startup.sh"]
