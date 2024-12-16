build:
	docker build -f Dockerfile_ros2_22_04 . -t aetherra --rm

cpu_run:
	docker run -itd --rm --ipc host --net host --pid host --name aetherra aetherra

gpu_run:
	docker run -itd --rm --net=host --pid host --ipc host --gpus all \
    --env=\"NVIDIA_DRIVER_CAPABILITIES=all\" --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v /dev:/dev \
	-v /home/animesh/workspaces/aetherra_ws:/home/aetherra_ws \
	--name aetherra aetherra

start:
	docker start aetherra

stop:
	docker stop aetherra

kill:
	docker kill aetherra

attach:
	docker exec -it aetherra bash

commit:
	docker commit aetherra aetherra2
	docker kill aetherra
	docker tag aetherra2 aetherra
