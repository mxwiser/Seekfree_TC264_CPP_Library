################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../code/route_process/route_image.cpp \
../code/route_process/route_process.cpp 

COMPILED_SRCS += \
code/route_process/route_image.src \
code/route_process/route_process.src 

CPP_DEPS += \
code/route_process/route_image.d \
code/route_process/route_process.d 

OBJS += \
code/route_process/route_image.o \
code/route_process/route_process.o 


# Each subdirectory must supply rules for building sources it contributes
code/route_process/route_image.src: ../code/route_process/route_image.cpp code/route_process/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Projects/TCCAR/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/route_process/route_image.o: code/route_process/route_image.src code/route_process/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/route_process/route_process.src: ../code/route_process/route_process.cpp code/route_process/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Projects/TCCAR/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/route_process/route_process.o: code/route_process/route_process.src code/route_process/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-route_process

clean-code-2f-route_process:
	-$(RM) code/route_process/route_image.d code/route_process/route_image.o code/route_process/route_image.src code/route_process/route_process.d code/route_process/route_process.o code/route_process/route_process.src

.PHONY: clean-code-2f-route_process

