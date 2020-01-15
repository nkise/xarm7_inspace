FROM nvidia/opengl:1.0-glvnd-devel-ubuntu16.04

WORKDIR /root

RUN apt-get update && apt-get install -y \
	wget \
	libglib2.0-0  \
	libgl1-mesa-glx \
	xcb \
	"^libxcb.*" \
	libx11-xcb-dev \
	libxkbcommon-x11-dev \
	libglu1-mesa-dev \
	libxrender-dev \
	libxi6 \
	libdbus-1-3 \
	libfontconfig1 \
	xvfb \
	libavcodec-dev \
	libavformat-dev \
	libswscale-dev

RUN wget http://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz 
RUN mkdir V-REP && \
	tar -xf CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz -C V-REP --strip-components=1 && \
	rm CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz

RUN echo 'export QT_DEBUG_PLUGINS=1' >> ~/.bashrc
RUN echo 'export PATH=/root/V-REP:$PATH' >> ~/.bashrc