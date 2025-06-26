cmake_minimum_required(VERSION 3.20)

project(examples)

add_executable(client chat_client.cpp)
target_link_libraries(client PRIVATE toolkit)
add_executable(server chat_server.cpp)
target_link_libraries(server PRIVATE toolkit)
