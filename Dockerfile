FROM etswalkingmachine/ros2-base:latest

RUN mkdir -p ~/dev/src/sr2_arm

COPY . /root/dev/src/

RUN apt-get update
RUN bash src/