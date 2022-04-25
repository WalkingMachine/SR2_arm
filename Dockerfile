FROM etswalkingmachine/sr2_interfaces

RUN mkdir -p ~/dev/src/sr2_arm

COPY . /root/dev/src/sr2_arm

RUN pip install -r /root/dev/src/sr2_arm/requirements.txt

RUN rosdep install -i --from-path src --rosdistro foxy -y

RUN colcon build --symlink-install --packages-select sr2_arm