FROM etswalkingmachine/ros2-base:latest

RUN mkdir -p ~/dev/src/sr2_arm

COPY . /root/dev/src/

RUN bash -c python3 -m pip install -r src/sr2_arm/requirements.txt

RUN rosdep install -i --from-path src --rosdistro foxy -y

RUN colcon build --symlink-install --packages-select sr2_arm