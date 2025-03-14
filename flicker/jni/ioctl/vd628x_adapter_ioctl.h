/********************************************************************************
Copyright (c) 2025, STMicroelectronics - All Rights Reserved
This file is licensed under open source license ST SLA0103
********************************************************************************/
#ifndef __VD628x_ADAPTER_IOCTL__
#define __VD628x_ADAPTER_IOCTL__ 1

#define VD628x_IOCTL_REG_WR		_IOW('r', 0x01, struct vd628x_reg)
#define VD628x_IOCTL_REG_RD		_IOWR('r', 0x02, struct vd628x_reg)

#define VD628x_IOCTL_GET_SPI_INFO	_IOWR('r', 0x01, struct vd628x_spi_info)
#define VD628x_IOCTL_SET_SPI_PARAMS	_IOW('r', 0x02, struct vd628x_spi_params)
#define VD628x_IOCTL_GET_CHUNK_SAMPLES	_IOWR('r', 0x03, __u16)

struct vd628x_reg {
	__u8 index;
	__u8 data;
};

struct vd628x_spi_info {
	__u32 chunk_size;
	__u32 spi_max_frequency;
};

struct vd628x_spi_params {
	__u32 speed_hz;
	__u16 samples_nb_per_chunk;
	__u16 pdm_data_sample_width_in_bytes;
};


#endif
