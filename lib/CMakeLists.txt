cmake_minimum_required(VERSION 3.0)

idf_component_register(
    INCLUDE_DIRS "./src/"
    REQUIRES driver esp_timer
)
