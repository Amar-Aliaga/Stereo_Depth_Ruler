FROM ubuntu:24.04


ENV DEBIAN_FRONTEND=noninteractive


RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential cmake git \
    libopencv-dev libopencv-contrib-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    nvidia-cuda-toolkit


RUN apt-get update && \
    apt-get install -y wget lsb-release zstd

COPY "/home/amar-aliaga/Desktop/my_video/output.mp4"


RUN apt-get update && apt-get install -y wget lsb-release

COPY ZED_SDK_Ubuntu24_cuda12.8_tensorrt10.9_v5.0.6.zstd.run /tmp/ZED_SDK.run

RUN chmod +x /tmp/ZED_SDK.run && \
    /tmp/ZED_SDK.run -- silent && \
    rm /tmp/ZED_SDK.run

WORKDIR /app


COPY . .


RUN rm -rf build && mkdir build && cd build && cmake .. && make -j$(nproc)


CMD ["./build/app/stereo_ruler"]
