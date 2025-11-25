FROM rust:1.91-bookworm AS build

WORKDIR /src

COPY ./server /src/server
COPY ./machine_interface /src/machine_interface
COPY ./dispatcher /src/dispatcher
COPY ./dparser /src/dparser
COPY ./dandelion_commons /src/dandelion_commons
COPY ./Cargo.toml /src/Cargo.toml
COPY ./Cargo.lock /src/Cargo.lock

RUN cargo build --bin dandelion_server

COPY ./target/debug/dandelion_server /src/target/debug/dandelion_server

FROM scratch

COPY --from=build /src/target/debug/dandelion_server /testa
COPY --from=build /lib/x86_64-linux-gnu/libc.so.6 /lib/x86_64-linux-gnu/libc.so.6
COPY --from=build /lib/x86_64-linux-gnu/libm.so.6 /lib/x86_64-linux-gnu/libm.so.6
COPY --from=build /lib/x86_64-linux-gnu/libgcc_s.so.1 /lib/x86_64-linux-gnu/libgcc_s.so.1
COPY --from=build /lib64/ld-linux-x86-64.so.2 /lib64/ld-linux-x86-64.so.2
