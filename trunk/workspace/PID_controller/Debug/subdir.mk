################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PmodCtlSys.c \
../pwm_tmrctr.c \
../test_PmodCtlSys_r2.c 

OBJS += \
./PmodCtlSys.o \
./pwm_tmrctr.o \
./test_PmodCtlSys_r2.o 

C_DEPS += \
./PmodCtlSys.d \
./pwm_tmrctr.d \
./test_PmodCtlSys_r2.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo Building file: $<
	@echo Invoking: MicroBlaze gcc compiler
	mb-gcc -Wall -O0 -g3 -c -fmessage-length=0 -Wl,--no-relax -I../../standalone_bsp_0/microblaze_0/include -mxl-barrel-shift -mxl-pattern-compare -mcpu=v8.40.b -mno-xl-soft-mul -mhard-float -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo Finished building: $<
	@echo ' '


