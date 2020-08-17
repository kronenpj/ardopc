cmake_minimum_required(VERSION 3.7)

# set the project name
set(PROJECT_NAME ardopc)
project(${PROJECT_NAME} C)

# add the executable
set(COMMON_SRC 
    ofdm.c
    KISSModule.c
    pktARDOP.c
    pktSession.c
    BusyDetect.c
    i2cDisplay.c
    ARDOPC.c
    ardopSampleArrays.c
    ARQ.c
    FFT.c
    FEC.c
    HostInterface.c
    Modulate.c
    rs.c
    berlekamp.c
    galois.c
    SoundInput.c
    TCPHostInterface.c
    SCSHostInterface.c)

if (WIN32)
    set(PLATFORM_SRC Waveout.c)
endif ()

if(UNIX AND NOT APPLE)
    set(PLATFORM_SRC
        ALSASound.c
        LinSerial.c)
endif()

if(APPLE)
    set(PLATFORM_SRC
    coreaudio.c
    LinSerial.c)
endif()

set(SRC_FILES ${COMMON_SRC} ${PLATFORM_SRC})

add_executable(${PROJECT_NAME} ${SRC_FILES})

if(UNIX AND NOT APPLE)
    find_library(ALSA_LIB asound)
    target_link_libraries(${PROJECT_NAME} PRIVATE "${ALSA_LIB}" rt m pthread)
endif()

