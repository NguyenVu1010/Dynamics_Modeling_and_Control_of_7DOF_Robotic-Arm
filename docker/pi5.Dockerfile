# Raspberry Pi 5 like image (design 11.5, ADR-017): Ubuntu 24.04 arm64, only apt packages the
# core uses, Cortex-A76 code generation. Build with
#   docker buildx build --platform linux/arm64 -f docker/pi5.Dockerfile --target runtime .
# Timing under QEMU is meaningless; the `test` stage checks correctness only.
FROM ubuntu:24.04 AS build
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      cmake ninja-build g++ libeigen3-dev libgtest-dev libgmock-dev libtinyxml2-dev \
      libyaml-cpp-dev python3 python3-yaml python3-numpy python3-pandas ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY CMakeLists.txt ./
COPY cmake cmake
COPY tests tests
COPY config config
COPY src/core src/core
COPY src/bench src/bench
COPY src/eval src/eval
COPY src/sim/urdf src/sim/urdf
COPY tools tools
RUN cmake -S . -B /build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCOBOT_WERROR=ON \
      "-DCOBOT_ARCH_FLAGS=-mcpu=cortex-a76" -DCMAKE_INSTALL_PREFIX=/opt/cobot \
    && cmake --build /build --parallel "$(nproc)" \
    && cmake --install /build

FROM build AS test
# Unit/component tests, then the bench e2e scenarios tagged for arm64 (E2E-08).
RUN ctest --test-dir /build --output-on-failure --parallel "$(nproc)" \
    && if ls tests/e2e/scenarios/*.yaml >/dev/null 2>&1; then \
         python3 tools/run_e2e.py --all --tag arm64 --bench /build/src/bench/cobot_bench \
           --out /tmp/e2e --no-report; \
       else echo "no e2e scenarios yet"; fi

FROM ubuntu:24.04 AS runtime
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      libtinyxml2-10 libyaml-cpp0.8 \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --uid 1001 cobot
COPY --from=build /opt/cobot /opt/cobot
COPY --from=build /src/config /opt/cobot/share/cobot/config
COPY --from=build /src/src/sim/urdf /opt/cobot/share/cobot/src/sim/urdf
ENV PATH=/opt/cobot/bin:$PATH
USER cobot
WORKDIR /home/cobot
CMD ["bash", "-c", "ls /opt/cobot/bin 2>/dev/null; echo 'cobot runtime image'"]
