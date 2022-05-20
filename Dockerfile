FROM ros:noetic-ros-core

SHELL ["/bin/bash", "-c"] 

RUN apt-get update \
    && apt-get install -y \
    git python3-pip python3-tf2-ros ros-noetic-foxglove-msgs \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install matplotlib notebook numpy==1.19 python-dateutil==2.8.2 nuscenes-devkit opencv-python-headless seaborn
RUN pip3 install git+https://github.com/DanielPollithy/pypcd.git

# RUN source /usr/local/bin/virtualenvwrapper.sh
RUN mkdir /notebooks

WORKDIR /notebooks
EXPOSE 8888/tcp

CMD ["jupyter", "notebook", "--port=8888", "--no-browser", "--ip=0.0.0.0", "--allow-root"]
