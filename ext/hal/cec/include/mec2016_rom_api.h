/*****************************************************************************
* © 2015 Microchip Technology Inc. and its subsidiaries.
* You may use this software and any derivatives exclusively with
* Microchip products.
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".
* NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
* TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
* CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
* FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
* OF THESE TERMS.
*****************************************************************************/



#ifndef INCLUDE_MEC2016_ROM_API_H_
#define INCLUDE_MEC2016_ROM_API_H_

#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/* QMSPI */
/**
 *  \spi_port_sel
 *
 *  \param [in] port Parameter_Description
 *  \param [in] en Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern void
spi_port_sel(uint8_t port, bool en);

/**
 *  \spi_port_drv_slew
 *
 *  \param [in] port Parameter_Description
 *  \param [in] drv_slew Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern void
spi_port_drv_slew(uint8_t port, uint8_t drv_slew);

/** rom_dis_lock_shd_spi - Apply GPIO Locks as specified in
 * customer section of EFUSE.
 * @param uint8_t 0(do not modify lock values), 1(insure
 * Shared SPI GPIO's are disabled (tri-state input) and
 * these pins are locked.
 * @note Disable all six QMSPI Shared SPI GPIO pins by
 * writing their GPIO Control registers to 0x0000_0040
 * which is GPIO, input, no-PUD, no interrupt detect,
 * no-invert, VTR power gate.
 * Lock all six QMSPI Shared SPI GPIO pin.
 * If EFUSE GPIO Lock values have bit[31]==1 then the
 * register was not written during ROM EFUSE processing and
 * this routine will write the value. If caller set lock parameter
 * to non-zero then we bit-wise OR the bits for Share SPI GPIO pins.
 *
 * Lock0: GPIO_0000 - GPIO_0036
 * Lock1: GPIO_0040 - GPIO_0076
 * Lock2: GPIO_0100 - GPIO_0136
 * Lock3: GPIO_0140 - GPIO_0176
 * Lock4: GPIO_0200 - GPIO_0236
 * Lock5: GPIO_0240 - GPIO_0276
 *
 * QMSPI Shared SPI pins are:   Lock Reg    Bit     byte    bit in byte
 * gpio_ctrl_055 = chip select  1           13      1       5
 * gpio_ctrl_056 = clock        1           14      1       6
 * gpio_ctrl_223 = IO0          4           19      2       3
 * gpio_ctrl_224 = IO1          4           20      2       4
 * gpio_ctrl_227 = IO2          4           23      2       7
 * gpio_ctrl_016 = IO3          0           14      1       6
 *
 *
 */
extern void rom_dis_lock_shd_spi(uint8_t lock_shd_spi);

/**
 *  \qmspi_init
 *
 *  \param [in] freq_hz Parameter_Description
 *  \param [in] spi_signalling Parameter_Description
 *  \param [in] ifctrl Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern void
qmspi_init(uint32_t freq_hz, uint8_t spi_signalling, uint8_t ifctrl);

/**
 *  \qmspi_freq_get
 *
 *  \return Return_Description
 *
 *  \details Details
 */
extern uint32_t
qmspi_freq_get(void);

/**
 *  \qmspi_freq_set
 *
 *  \param [in] freq_hz Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern void
qmspi_freq_set(uint32_t freq_hz);

/**
 *  \qmspi_xfr_done_status
 *
 *  \param [in] hw_status Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern bool
qmspi_xfr_done_status(uint32_t* hw_status);

/**
 *  \qmspi_start
 *
 *  \param [in] ien_mask Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern void
qmspi_start(uint16_t ien_mask);

/**
 *  \qmspi_start_dma
 *
 *  \param [in] dma_chan Parameter_Description
 *  \param [in] ien_mask Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern void
qmspi_start_dma(uint8_t dma_chan, uint16_t ien_mask);

/**
 *  \qmspi_cfg_spi_cmd
 *
 *  \param [in] spi_cmd Parameter_Description
 *  \param [in] spi_address Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern uint8_t
qmspi_cfg_spi_cmd(uint32_t spi_cmd, uint32_t spi_address);

/**
 *  \qmspi_read_fifo
 *
 *  \param [in] pdata Parameter_Description
 *  \param [in] byte_len Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern uint32_t
qmspi_read_fifo(uint8_t* pdata, uint32_t byte_len);

/**
 *  \qmspi_read_dma
 *
 *  \param [in] spi_cmd Parameter_Description
 *  \param [in] spi_address Parameter_Description
 *  \param [in] mem_addr Parameter_Description
 *  \param [in] nbytes Parameter_Description
 *  \param [in] dma_chan Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern uint32_t
qmspi_read_dma(uint32_t spi_cmd, uint32_t spi_address,
					   uint32_t mem_addr, uint32_t nbytes, uint8_t dma_chan);

/**
 *  \qmspi_write_dma
 *
 *  \param [in] spi_cmd Parameter_Description
 *  \param [in] spi_address Parameter_Description
 *  \param [in] mem_addr Parameter_Description
 *  \param [in] nbytes Parameter_Description
 *  \param [in] dma_chan Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern uint32_t
qmspi_write_dma(uint32_t spi_cmd, uint32_t spi_address,
						uint32_t mem_addr, uint32_t nbytes, uint8_t dma_chan);

/**
 *  \qmspi_xmit_cmd
 *
 *  \param [in] cmd_params Parameter_Description
 *  \param [in] ntx Parameter_Description
 *  \param [in] nresponse Parameter_Description
 *  \return Return_Description
 *
 *  \details Details
 */
extern bool
qmspi_xmit_cmd(uint8_t* cmd_params, uint8_t ntx, uint8_t nresponse);

/* RNG */
/**
 *  \rng_power
 *
 *  \param [in] pwr_on Power On?
 *  \return none
 *
 *  \details Gate clocks on/off to NDRNG block
 */
extern void
rng_power(bool pwr_on);


/**
 *  \rng_reset
 *
 *  \return Reset NDRNG block
 *
 *  \details
 */
extern void
rng_reset(void);


/**
 *  \rng_mode
 *
 *  \param [in] mode tmode_pseudo 0(asynchronous/true random mode),
 *              Non-zero(pseudo-random mode)
 *  \return None
 *
 *  \details Set NDRNG random number generation mode
 */
extern void
rng_mode(uint8_t mode);


/**
 *  \rng_is_on
 *
 *  \return is NDRNG Block powered on? True if yes, false otherwise
 *
 *  \details Check if NDRNG block is powered on.
 */
extern bool
rng_is_on(void);


/**
 *  \rng_start
 *
 *  \return None
 *
 *  \details Start NDRNG engine
 */
extern void
rng_start(void);

/**
 *  \rng_stop
 *
 *  \return Void
 *
 *  \details Stop NDRNG engine
 */
extern void
rng_stop(void);


/**
 *  \rng_get_fifo_level
 *
 *  \return actual number of 32-bit words in the NDRNG FIFO.
 *
 *  \details return the number of 32-bit words of random data
 * currently in the FIFO.
 */
extern uint32_t
rng_get_fifo_level(void);


/**
 *  \rng_get_bytes
 *
 *  \param [in] pbuff Output Buffer
 *  \param [in] nbytes Number of bytes to be read
 *  \return Number of bytes retrieved
 *
 *  \details read bytes from the NDRNG FIFO
 */
extern uint32_t
rng_get_bytes(uint8_t* pbuff, uint32_t nbytes);


/**
 *  \rng_get_words
 *
 *  \param [in] pwords Pointer to output buffer
 *  \param [in] nwords Number of words to read
 *  \return actual number of words read
 *
 *  \details Details
 */
extern uint32_t
rng_get_words(uint32_t* pwords, uint32_t nwords);


/* AES */
/**
 *  \aes_hash_power
 *
 *  \param [in] pwr_on Gate/Ungate clocks to block
 *  \return None
 *
 *  \details Enable/Disable AES and HASH H/W Block
 */
extern void
aes_hash_power(uint8_t pwr_on);

/**
 *  \aes_hash_reset
 *
 *  \return None
 *
 *  \details Stop AES and HASH
 */
extern void
aes_hash_reset(void);

/**
 *  \aes_busy
 *
 *  \return Is AES Block Running? True if yes, false Otherwise.
 *
 *  \details Is AES Block Running?
 */
extern bool
aes_busy(void);


/**
 *  \aes_status
 *
 *  \return Status of AES Block
 *
 *  \details Returns the Status of AES Block
 */
extern uint32_t
aes_status(void);

/**
 *  \aes_done_status
 *
 *  \param [in] hw_status Pointer to where the status value will be updated
 *  \return True if done, false otherwise
 *
 *  \details Returns the done status of AES block
 */
extern bool
aes_done_status(uint32_t* hw_status);

/**
 *  \aes_stop
 *
 *  \return Return aes_busy() Status
 *
 *  \details Stop AES Operations
 */
extern bool
aes_stop(void);

/**
 *  \aes_start
 *
 *  \param [in] ien Enable interrupts?
 *  \return None
 *
 *  \details Start AES block with or without interrupts
 */
extern void
aes_start(bool ien);

/**
 *  \aes_iclr
 *
 *  \return Status of the AES Block
 *
 *  \details Clears AES Hash Interrupts
 */
extern uint32_t
aes_iclr(void);


/**
 *  \brief Brief
 *
 *  \param [in] pkey Aligned buffer with AES Key
 *  \param [in] piv Aligned buffer with AES initialization
 *  \param [in] key_len AES_KEYLEN_128, AES_KEYLEN_192, AES_KEYLEN_256
 *  \param [in] msbf Most Significant Byte order first?
 *  \return AES_ERR_BAD_POINTER, AES_ERR_BAD_KEY_LEN, AES_OK
 *
 *  \details Load AES Accelerator with key and optional Initialization vector
 */
extern uint8_t
aes_set_key(const uint32_t* pkey,
				const uint32_t* piv,
				uint8_t key_len, bool msbf);

/**
 *  \aes_crypt
 *
 *  \param [in] data_in Aligned input data Buffer
 *  \param [in] data_out Aligned output data buffer
 *  \param [in] num_128bit_blocks Size of input in 16-byte blocks
 *  \param [in] mode AES Encryption/Decryption Mode
 *  \return AES_OK, AES_ERR_BAD_POINTER,
 *
 *  \details Program specified AES Operation using currently programmed key
 */
extern uint8_t
aes_crypt(const uint32_t* data_in,
			  uint32_t* data_out,
			  uint32_t num_128bit_blocks, uint8_t mode);


/* SHA */
#define SHA1_BLEN           (20u)
#define SHA1_WLEN           (5u)
#define SHA2_BLEN           (32u)
#define SHA2_WLEN           (8u)
#define SHA12_BLOCK_BLEN    (64u)
#define SHA12_BLOCK_WLEN    (16u)
#define SHA3_BLEN           (48u)
#define SHA3_WLEN           (12u)
#define SHA5_BLEN           (64u)
#define SHA5_WLEN           (16u)
#define SHA35_BLOCK_BLEN    (128u)
#define SHA35_BLOCK_WLEN    (32u)

/* return values */
#define SHA_RET_OK                      (0) /* OK */
#define SHA_RET_START                   (1) /* OK, SHA Engine started */
#define SHA_RET_ERROR                   (0x80)  /* b[7]==1 indicates an error */
#define SHA_RET_ERR_BUSY                (0x80)
#define SHA_RET_ERR_BAD_ADDR            (0x81)
#define SHA_RET_ERR_TIMEOUT             (0x82)
#define SHA_RET_ERR_MAX_LEN             (0x83)
#define SHA_RET_ERR_UNSUPPORTED         (0x84)

#define SHA_MODE_MD5    (0) // Not supported by HW
#define SHA_MODE_1      (1)
#define SHA_MODE_224    (2) // Not supported by HW
#define SHA_MODE_256    (3)
#define SHA_MODE_384    (4) // Not supported by HW
#define SHA_MODE_512    (5)

#define HASH_START_IEN      (1u)
#define HASH_START_NOIEN    (0u)

typedef union {
    uint32_t w[SHA2_WLEN];
    uint8_t  b[SHA2_BLEN];
} SHA12_DIGEST_U;

typedef union {
    uint32_t w[SHA5_WLEN];
    uint8_t  b[SHA5_BLEN];
} SHA35_DIGEST_U;

/*
 * !!! SHA-1 & SHA-256
 * HW Engine requires alignment >= 4-byte boundary !!!
 */
typedef struct sha12_context_s SHA12_CONTEXT_T;
struct sha12_context_s {
    SHA12_DIGEST_U hash;
    union {
        uint32_t w[(SHA12_BLOCK_WLEN) * 2];
        uint8_t  b[(SHA12_BLOCK_BLEN) * 2];
    } block;
    uint8_t mode;
    uint8_t block_len;
    uint8_t rsvd[2];
    uint32_t total_msg_len;
};

/*
 * !!! SHA-512 HW Engine requires alignment >= 8-byte boundary !!!
 */
typedef struct sha35_context_s SHA35_CONTEXT_T;
struct sha35_context_s {
    SHA35_DIGEST_U hash;
    union {
        uint32_t w[(SHA35_BLOCK_WLEN) * 2];
        uint8_t  b[(SHA35_BLOCK_BLEN) * 2];
    } block;
    uint8_t mode;
    uint8_t block_len;
    uint8_t rsvd[2];
    uint32_t total_msg_len;
};


/**
 *  \hash_busy
 *
 *  \return is busy? True if yes, Flase other wise
 *
 *  \details returns the busy status of Hash Block
 */
extern bool hash_busy(void);

/**
 *  \hash_start
 *
 *  \param [in] ien enable/disable interrupts
 *  \return None
 *
 *  \details start hash block
 */
 extern void
hash_start(bool ien);

/**
 *  \hash_done_status
 *
 *  \param [in] hw_status Hash Status Register Value
 *  \return true if done, false otherwise
 *
 *  \details reflects the done status of HASH black and updates
 *		status regsiter value into the input variable
 */
extern bool
hash_done_status(uint32_t* hw_status);

/**
 *  \sha12_init
 *
 *  \param [in] psha12_ctx Data Structure for Input data and 	Output Digest
 *  \param [in] mode SHA_MODE_1 or SHA_MODE_256
 *  \return SHA_RET_ERR_BAD_ADDR, SHA_RET_ERR_UNSPPORTED ,SHA_RET_OK
 *
 *  \details Initializes the Data structure provided
 */
extern uint8_t
sha12_init(SHA12_CONTEXT_T* psha12_ctx, uint8_t mode);

/**
 *  \sha12_update
 *
 *  \param [in] psha12_ctx Data Structure for Input data and Output Digest
 *  \param [in] pdata Input Data to Hash Block
 *  \param [in] num_bytes Byte length of input data
 *  \return SHA_RET_ERR_BAD_ADDR, SHA_RET_ERR_BUSY, SHA_RET_ERR_MAX_LEN, SHA_RET_OK
 *
 *  \details Run hash block on data and if data greater than block size, put remaining bytes back into the data structure
 */
extern uint8_t
sha12_update(SHA12_CONTEXT_T* psha12_ctx,
				 const uint32_t* pdata, uint32_t num_bytes);

/**
 *  \sha12_finalize
 *
 *  \param [in] psha12_ctx Data Structure for Input data and Output Digest
 *  \return SHA_RET_ERR_BAD_ADDR, SHA_RET_ERR_BUSY ,SHA_RET_START
 *
 *  \details Apply FIPS padding to SHA256 and perform final hash calculation.
 */
extern uint8_t
sha12_finalize(SHA12_CONTEXT_T* psha12_ctx);

/**
 *  \sha35_init
 *
 *  \param [in] psha35_ctx Data structure for input message and Digest output
 *  \param [in] mode SHA_MODE_512
 *  \return SHA_RET_ERR_BAD_ADDR, SHA_RET_ERR_UNSUPPORTED ,SHA_RET_ERR_BUSY, SHA_RET_OK
 *
 *  \details Initialize the data structure provided
 */
extern uint8_t
sha35_init(SHA35_CONTEXT_T* psha35_ctx, uint8_t mode);

/**
 *  \sha35_update
 *
 *  \param [in] psha35_ctx Data structure for input message and Digest output
 *  \param [in] pdata Input to Hash block
 *  \param [in] num_bytes size of input in bytes
 *  \return SHA_RET_ERR_BAD_ADDR, SHA_RET_ERR_BUSY, SHA_RET_OK
 *
 *  \details Run Hash block on data and copy remaining bytes (if any) back into the data structure
 */
extern uint8_t
sha35_update(SHA35_CONTEXT_T* psha35_ctx,
		         const uint32_t* pdata, uint32_t num_bytes);


/**
 *  \sha35_finalize
 *
 *  \param [in] psha35_ctx Data structure for input message and Digest output
 *  \return SHA_RET_ERR_BAD_ADDR, SHA_RET_ERR_BUSY, SHA_RET_OK
 *
 *  \details Apply FIPS padding to SHA256 and perform final hash calculation.
 */
extern uint8_t
sha35_finalize(SHA35_CONTEXT_T* psha35_ctx);


/**
 *  \hash_iclr
 *
 *  \return Hash Block status
 *
 *  \details Clear Hash Interrupt
 */
extern uint32_t
hash_iclr(void);


/**
 *  \sha_init
 *
 *  \param [in] mode SHA_MODE_1, SHA_MODE_256, SHA_MODE_512
 *  \param [in] pdigest Address where digest will be stored
 *  \return * 	0 = Success
 * 				1 = Hash Engine busy
 * 				2 = Unsupported SHA operation
 * 				3 = Bad digest pointer, NULL or mis-aligned.
 *  \details 	Initialize Hash engine for SHA operation.
 * 				Programs supported SHA operation's initial value, digest address,
 * 				and operation
 */
extern uint8_t
sha_init(uint8_t mode, uint32_t* pdigest);


/**
 *  \sha_update
 *
 *  \param [in] pdata Input Data
 *  \param [in] nblocks Size in blocks
 *  \param [in] flags bit(0) - Clear Status?, bit(1) - Enable Interrupts?, bit(2) - Start?
 *  \return 0 - OK, 1 - Hash Busy, 2 - bad address for data, 3 - Buffer not aligned
 *
 *  \details Run Hash block on data
 */
extern uint8_t
sha_update(uint32_t* pdata, uint16_t nblocks, uint8_t flags);


/**
 *  \sha_final
 *
 *  \param [in] padbuf Buffer for padding (Twice block size)
 *  \param [in] total_msg_len Message length in bytes
 *  \param [in] prem Parameter_Description
 *  \param [in] flags bit(0) - Clear Status?, bit(1) - Enable Interrupts?, bit(2) - Start?
 *  \return 0 - OK, 1 - Hash Busy, 2 - bad address for data, 3 - Buffer not aligned
 *
 *  \details Run final SHA Calculations and add padding
 */
extern uint8_t
sha_final(uint32_t* padbuf, uint32_t total_msg_len,
		      const uint8_t* prem, uint8_t flags);


/* PKE Miscellaneous */

#define PKE_RET_STARTED                         (0)
#define PKE_RET_OK                              (0)
#define PKE_RET_ERR_BUSY                        (1)
#define PKE_RET_ERR_BAD_PARAM                   (2)
#define PKE_RET_ERR_BAD_ADDR                    (3)
#define PKE_RET_ERR_UNKNOWN_OP                  (4)
#define PKE_RET_ERR_INVALID_BIT_LENGTH          (5)
#define PKE_RET_ERR_INVALID_MSG_LENGTH          (6)

typedef struct buff8_s
{
    uint32_t len;
    uint8_t *pd;
} BUFF8_T;

/**
 *  \pke_power
 *
 *  \param [in] pwr_on power on?
 *  \return None
 *
 *  \details Gate or Ungate power to PKE block
 */
extern void
pke_power(bool pwr_on);


/**
 *  \brief pke_reset
 *
 *  \return None
 *
 *  \details Reset PKE Block
 */
extern void
pke_reset(void);

/**
 *  \pke_status
 *
 *  \return Return PKE Status register value
 *
 *  \details Details
 */
extern uint32_t
pke_status(void);

/**
 *  \pke_done_status
 *
 *  \param [in] hw_status POinter where PKE Status is updated
 *  \return True if done, false otherwise
 *
 *  \details Returns the done status of PKE block
 */
extern bool
pke_done_status(uint32_t* hw_status);

/**
 *  \pke_start
 *
 *  \param [in] ien Interrupt Enable?
 *  \return None
 *
 *  \details Start PKE Block
 */
extern void
pke_start(bool ien);


/**
 *  \pke_busy
 *
 *  \return Busy? True if busy, false otherwise
 *
 *  \details Details
 */
extern bool
pke_busy(void);

/**
 * pke_ists_clear - Read and clear PKE status
 *
 * @return PKE status register in bits[16:0], GIRQ16.Source[1:0]
 *         in bits[21:20].
 *
 * @note Reading PKE Status register clears block interrupt
 *       signals. Reading does not clear status bits. PKE Status
 *       bits[31:17] are unused. We put PKE GIRQ16 bits[1:0]
 *       into bits [21:20]
 */
extern uint32_t pke_ists_clear(void);


/**
 *  \pke_clear_scm
 *
 *  \return None
 *
 *  \details Clear the Shared Crypto memory
 */
extern void
pke_clear_scm(void);

/**
 *  \pke_scm_clear_slot
 *
 *  \param [in] slot_num Slot number in Shared Crypto Memory
 *  \return None
 *
 *  \details Clear the specified slot in Shared Crypto Memory
 */
extern void
pke_scm_clear_slot(uint8_t slot_num);

/**
 *  \pke_read_scm
 *
 *  \param [in] dest Pointer to where the data is to be read
 *  \param [in] nbytes Number of bytes to be read
 *  \param [in] slot_num Slot number from which data is to be read
 *  \param [in] reverse_byte_order Reverse Byte order? True if yes, false otherwise
 *  \return Number of bytes Read
 *
 *  \details Read data from specified slot number in Shared Crypto memory
 */
extern uint16_t
pke_read_scm(uint8_t* dest, uint16_t nbytes,
				 uint8_t slot_num, bool reverse_byte_order);


/**
 *  \pke_write_scm
 *
 *  \param [in] pdata Data to be written
 *  \param [in] num_bytes Number of bytes to be written
 *  \param [in] slot_num Slot number to which data ought to be written
 *  \param [in] reverse_byte_order Reverse Byte order? True if yes, false otherwise
 *  \return None
 *
 *  \details Write data provided to specified slot in Shared Crypto Memory
 */
extern void
pke_write_scm(const void* pdata, uint16_t num_bytes,
				  uint8_t slot_num, uint8_t reverse_byte_order);


/**
 *  \pke_set_operand_slot
 *
 *  \param [in] operand
 *  \param [in] slot_num Slot number for the selected operand
 *  \return None
 *
 *  \details Set the slot for the selected operand
 */
extern void pke_set_operand_slot(uint8_t operand, uint8_t slot_num);

/**
 *  \pke_get_operand_slot
 *
 *  \param [in] operand
 *  \return Slot number for the operand
 *
 *  \details Get the slot for the selected operand
 */
extern uint8_t pke_get_operand_slot(uint8_t operand);

/**
 *  \pke_set_operand_slots
 *
 *  \param [in] op_slots
 *  \return None
 *
 *  \details Set the slot numbers for operands P1, P2 and P3
 */
extern void pke_set_operand_slots(uint32_t op_slots);

/**
 *  \pke_get_slot_addr
 *
 *  \param [in] slot_num Slot number for whose address to be obtained
 *  \return Address
 *
 *  \details Get the address of Slot in the SCM
 */
extern uint32_t pke_get_slot_addr(uint8_t slot_num);

/**
 *  \pke_fill_slot
 *
 *  \param [in] slot_num Slot number
 *  \param [in] fill_val Value to be filled in the slot
 *  \return None
 *
 *  \details Fill the slot area with the fill_val
 */
extern void pke_fill_slot(const uint8_t slot_num, const uint32_t fill_val);

/* PKE RSA */

/**
 *  \rsa_load_key
 *
 *  \param [in] rsa_bit_len 1024, 2048, 4096
 *  \param [in] private_exponent Pointer to structure having length & pointer to private exponent
 *  \param [in] public_modulus Pointer to structure having length & pointer to Public modulus
 *  \param [in] public_exponent Pointer to structure having length & pointer to Public Exponent
 *  \param [in] msbf Reverse Byte order? True if yes, false otherwise
 *  \return PKE_RET_ERR_BUSY, PKE_RET_ERR_INVALID_BIT_LENGTH, PKE_RET_OK
 *
 *  \details Load RSA keys into Crypto memory
 */
extern uint8_t
rsa_load_key(uint16_t rsa_bit_len,
					 const BUFF8_T* private_exponent,
					 const BUFF8_T* public_modulus,
					 const BUFF8_T* public_exponent,
					 bool msbf);

/** RSA Modular Exponentiation C = M^e mod n
 * @param rsa_bit_len = 1024, 2048, or 4096 bits
 * @param pointer to BUFF8_T structure containing number to exponentiate
 * @param pointer to BUFF8_T structure containing exponent
 * @param pointer to BUFF8_T structure contianing modulus
 * @param boolean indicating byte ordering (false = LSBF, true = MSBF)
 * @return 0(success), non-zero(error code)
 * @note Parameters are loaded into SCM as follows:
 * OptPtrA specifies slot number of M = 1
 * OptPtrB specifies slot number of e = 2
 * OptPtrC specifies slot number of result, C = (M^e) mod n = Slot 3
 * n is located in Slot 0
*/
extern uint8_t rsa_modular_exp(uint16_t rsa_bit_len,
                                 const BUFF8_T *M,
                                 const BUFF8_T *e,
                                 const BUFF8_T *n,
                                 bool msbf);

/** RSA CRT Parameter Generation
 * @param rsa_bit_len = 1024, 2048, or 4096 bits
 * @param pointer to prime p
 * @param pointer to prime q
 * @param flags bit[0]=0(do not start), 1(start after programming)
 * bit[4] = byte order: 0 = Least significant byte first, 1 = Most significant byte first
 * bit[1]=0(do not enable interrupt), 1(enable interrupt before starting)
 * @return 0(success), non-zero(error code)
 * @note Requires RSA Keys to have been previously loaded, Public Modulus must
 * be in Slot 0 and Public exponent in Slot 8. Private Exponent must be in Slot 6.
 * Prime p will be loaded into Slot 2 and prime q into Slot 3. After the engine is
 * done, the three output parameters are: dp in Slot 0xA, dq in Slot 0xB, and
 * I in Slot 0xC.
 *
*/
extern uint8_t rsa_crt_gen_params(uint16_t rsa_bit_len,
                                    const BUFF8_T* p,
                                    const BUFF8_T* q,
                                    const BUFF8_T* pubmod,
                                    const BUFF8_T* prvexp,
                                    bool msbf);

/** RSA Decryption using Chinese Remainder Theorem
 * @param rsa_bit_len = 1024, 2048, or 4096 bits
 * @param pointer to encrypted message
 * @param encrypted message byte length
 * @param flags bit[0]=0(do not start), 1(start after programming)
 * bit[4] = byte order: 0 = Least significant byte first, 1 = Most significant byte first
 * bit[1]=0(do not enable interrupt), 1(enable interrupt before starting)
 * @return 0(success), non-zero(error code)
 * @note Computes M = (encrypted_mesg)^(Slot 6). Slot 6 contains either the
 * private or public exponent. If the message was signed with a private key
 * then Slot 6 should contain the public exponent. If the message was signed
 * with a public key then Slot 6 should contain the private exponent.
 * Switch the order of parameters in pke_rsa_load_keys() to store the appropriate
 * exponent in Slot 6. Slots 0xA, 0xB, and 0xC must contain the CRT parameters
 * dp, dq, and I respectively.
 * Unencrypted output will be in Slot 5 and will contain PKCS#1 v1.5 padding.
 *
*/
extern uint8_t rsa_crt_decrypt(uint16_t rsa_bit_len,
                                 const BUFF8_T* encrypted_mesg,
                                 bool msbf);

/** RSA Signature Generation
 * @param rsa_bit_len = 1024, 2048, or 4096 bits
 * @param pointer to Hash to sign. It is the caller's responsibility to properly pad the Hash.
 * @param hash length in bytes
 * @param flags bit[0]=0(do not start), 1(start after programming)
 * bit[4] = byte order: 0 = Least significant byte first, 1 = Most significant byte first
 * bit[1]=0(do not enable interrupt), 1(enable interrupt before starting)
 * @return 0(success), non-zero(error code)
 * @note Computes M = (hash)^(Slot 6). Slot 6 contains either the RSA key
 * private or public exponent. The opposite exponent must be used for signature
 * verification. Slot 0 contains the RSA key public modulus. The hash is loaded
 * into Slot 4. Signature output is in Slot 5. The hash digest is truncated to
 * (rsa_bit_len / 8) bytes.
 *
*/
extern uint8_t rsa_signature_gen(uint16_t rsa_bit_len,
                                   const BUFF8_T* hash_digest,
                                   bool msbf);

/** RSA Signature Verification
 * @param rsa_bit_len = 1024, 2048, or 4096 bits
 * @param pointer to signature
 * @param pointer to Hash to verify
 * @param hash length in bytes
 * @param flags bit[0]=0(do not start), 1(start after programming)
 * bit[4] = byte order: 0 = Least significant byte first, 1 = Most significant byte first
 * bit[1]=0(do not enable interrupt), 1(enable interrupt before starting)
 * @return 0(success), non-zero(error code)
 * @note Computes h = (signature)^(Slot 8). Slot 8 contains either the RSA key
 * private or public exponent. The opposite exponent must be used for signature
 * generation. Slot 0 contains the RSA key public modulus. The expected hash is
 * loaded into Slot 0xC. Recovered hash digest output is in Slot 5. PKE compares
 * the contents of Slot 5 with Slot 0xC. The recovered hash digest will also contain
 * PKCS#1 v1.5 padding. The expected hash digest must also contain the same padding.
 *
*/
extern uint8_t rsa_signature_verify(uint16_t rsa_bit_len,
                                      const BUFF8_T *signature,
                                      const BUFF8_T *hash_digest_pkcs15,
                                      bool msbf);

/**
 *  \rsa_encrypt
 *
 *  \param [in] rsa_bit_len 1024, 2048, 4096
 *  \param [in] mesg pointer to structure having length & pointer to Message to be encrypted
 *  \param [in] msbf Reverse Byte order? True if yes, false otherwise
 *  \return PKE_RET_ERR_BAD_ADDR, PKE_RET_ERR_BUSY, PKE_RET_ERR_INVALID_MSG_LENGTH, PKE_RET_ERR_INVALID_BIT_LENGTH, PKE_RET_OK
 *
 *  \details Encrypt provided message. Load Keys before this function is called
 */
extern uint8_t
rsa_encrypt(uint16_t rsa_bit_len,
					const BUFF8_T* mesg,
					bool msbf);



/**
 *  \rsa_decrypt
 *
 *  \param [in] rsa_bit_len 1024, 2048, 4096
 *  \param [in] encrypted_mesg pointer to structure having length & pointer to Encrypted data
 *  \param [in] msbf Reverse Byte order? True if yes, false otherwise
 *  \return PKE_RET_ERR_BAD_ADDR, PKE_RET_ERR_BUSY, PKE_RET_ERR_INVALID_MSG_LENGTH, PKE_RET_ERR_INVALID_BIT_LENGTH, PKE_RET_OK
 *
 *  \details Perform decryption on provided encrypted message. load keys before calling this function
 */
extern uint8_t
rsa_decrypt(uint16_t rsa_bit_len,
					const BUFF8_T* encrypted_mesg,
					bool msbf);

/** RSA Load CRT Key Parameters - Load RSA Key CRT parameters
  * crypto memory.
  * @param rsa_bit_len =  1024, 2048, or 4096 bits
  * @param pointer to RSA key CRT parameter dp (rsa_bit_len / 8) bytes
  * @param pointer to RSA key CRT parameter dq (rsa_bit_len / 8) bytes
  * @param pointer to RSA key CRT parameter I (rsa_bit_len / 8) bytes
  * @param flags bits[3:0] ignored, bit[4] = byte order
  * (0/false) = Least significant byte first,
  * (1/true) Most significant byte first
  * @return 0(success), non-zero(error)
  * @note If a parameter pointer is not NULL load it into its SCM slot.
  * Parameter dp loaded into Slot 0xA, Paramter dq loaded into Slot 0xB, and
  * Parameter I loaded into Slot 0xC.
  */
extern uint8_t rsa_load_crt_params(uint16_t rsa_bit_len,
                                     const BUFF8_T *dp,
                                     const BUFF8_T *dq,
                                     const BUFF8_T *I,
                                     bool msbf);

/** RSA Key Generation
 * @param rsa_bit_len = 1024, 2048, or 4096-bits
 * @param pointer to RSA prime p of length (rsa_bit_len / 8)
 * @param pointer to RSA prime q of length (rsa_bit_len / 8)
 * @param pointer to RSA public exponent
 * @param byte length of p & q
 * @param byte length of public exponent
 * @param flags bit[0]=0(do not start), 1(start after programming)
 * bit[1]=0(do not enable interrupt), 1(enable interrupt before starting)
 * bits[3:2]=0 reserved
 * bit[4] = byte order: 0 = Least significant byte first, 1 = Most significant byte first
 * @return 0(success), non-zero(error code)
 * @note RSA key generation does not use and ignores OptPtr's
 * in PKE Config register. Output is Private-Public key pair:
 * Slot 0 = Public modulus
 * Slot 6 = Private modulus
 * Public key is (Public Modulus, Public Exponent)
 * Private key is Private Modulus
*/

extern uint8_t rsa_key_gen(uint16_t rsa_bit_len,
                             const BUFF8_T *p,
                             const BUFF8_T *q,
                             const BUFF8_T *e,
                             bool msbf);


/* PKE ECDSA */

/* Order of parameter pointers in struct elliptic_curve param[] member
 * These indices correspond to the SCM slot numbers. This could change
 * in a newer PKE IP block. */
#define ELLIPTIC_CURVE_PARAM_P      (0u)
#define ELLIPTIC_CURVE_PARAM_N      (1u)
#define ELLIPTIC_CURVE_PARAM_GX     (2u)
#define ELLIPTIC_CURVE_PARAM_GY     (3u)
#define ELLIPTIC_CURVE_PARAM_A      (4u)
#define ELLIPTIC_CURVE_PARAM_B      (5u)
#define ELLIPTIC_CURVE_NPARAMS      (6u)

typedef struct elliptic_curve
{
    uint32_t *param[ELLIPTIC_CURVE_NPARAMS];
    uint16_t byte_len;
    uint8_t flags;
    uint8_t rsvd1;
} ELLIPTIC_CURVE;

/*
 *  $ec_prog_curve
 *
 *  $param [in] pcurve The elliptic Curve to be programmed into PKE
 *  $return ECC_ERR_BAD_PARAM, PKE_RET_OK
 *
 *  $details Programs an curve into the PKE engine
 */
extern uint8_t ec_prog_curve(const ELLIPTIC_CURVE* pcurve);

/*
 *  $ecdsa_verify
 *
 *  $param [in] Q      Pointer to public key
 *  $param [in] S      Pointer to signature
 *  $param [in] digest pointer to digest
 *  $param [in] elen   length of EC curve
 *  $param [in] dlen   digest length in bytes
 *  $param [in] msbf   most significant byte first?
 *  $return ECC_ERR_BAD_PARAM, ECC_ERR_BUSY, ECC_ERR_ZERO_LEN_PARAM, PKE_RET_OK
 *
 *  $details programmes the data required for verification
 */
extern uint8_t ecdsa_verify(const uint8_t* Q,
																const uint8_t* S,
																const uint8_t* digest,
																uint16_t elen,
																uint16_t dlen,
																bool msbf);


																/*
 * @param pointer to array containing EC Private key
 * @param byte length of d
 * @param flags bit[0]=0(d is LSBF), 1(d is MSBF)
 * @return 0=success(PKE started), non-zero=error(PKE not started)
 * @notes - Caller must have previously programmed elliptic curve
 * into PKE e.g. pke_ec_prog_curve().
 */
extern uint8_t ec_kcdsa_keygen(const uint8_t* d, uint16_t plen, uint16_t flags);

/*
 * @param pointer to array containing EC Private key
 * @param pointer to array containing r component of signature
 * @param byte length of each d and r
 * @param pointer to hash digest of message
 * @param byte length of hash digest
 * @param flags bit[0]=0(d is LSBF), 1(d is MSBF)
 *              bit[1]=0(r is LSBF), 1(r is MSBF)
 *              bit[2]=0(digest is LSBF), 1(digest is MSBF)
 * @return 0=success(PKE started), non-zero=error(PKE not started)
 * @notes - Caller must have previously programmed elliptic curve
 * into PKE e.g. pke_ec_prog_curve().
 */
extern uint8_t ec_kcdsa_sign(const uint8_t* prv_key, uint16_t plen,
                               const uint8_t* r, uint16_t rlen,
                               const uint8_t* hash, uint16_t hlen,
                               uint16_t flags);

/** pke_ec_prog_curve - Program elliptic curve parameters into PKE
 *  shared crypto memory.
 * @param pointer to ELLIPTIC_CURVE structure
 * @return 0 = success, non-zero is error code.
 * @note Programs EC parameters, prime to slot0, order to slot 1,
 * generator point x-coord. to slot 2, generator point y-coord to
 * slot 3, curve parameter a to slot 4, and curve parameter b to
 * slot 5. All parameters are zero extended to the end of the slot.
 * Current MEC1322 PKE slot size is 256 bytes.
 * MEC2016 PKE slot size is 512 bytes.
 * This routine also
 * programs PKE register fields: Command register
 *  Operand size field (bits[15:8])
 *   PKE EC requires operand size to be a multiple of 16 bytes.
 *   PKE docs state "number of 64-bit double words" which we interpret
 *   as 128-bits (16-bytes) based on values used. The size field is in
 *   units of 64-bits. P-192,224,256 use 0x04(4 * 64 = 256 a multiple of 16)
 *   P-384 uses 0x06 (6 * 64 = 384 a multiple of 16)
 *   P-521 uses 0x0A (10 * 64 = 640 a multiple of 16 but 521 bits is 66 bytes not
 *   a multiple of 16). The next multiple of 16 is 640 (0x0A).
 *  Field bit[7] = 0 F(p) or 1 F(2m).
 * NOTE: MEC1322 PKE does not support F(2m) binary curves.
 */
extern uint8_t ec_prog_curve( const ELLIPTIC_CURVE* curve_p );

/*
 * @param pointer to array containing Qx and Qy
 * @param pointer to array containing r and s
 * @param byte length of each Qx, Qy, r, and s.
 * @param pointer to hash digest of message
 * @param byte length of hash digest
 * @param flags bit[0]=0(Qx,y are LSBF), 1(Qx,y are MSBF)
 *              bit[1]=0(r,s are LSBF), 1(r,s are MSBF)
 *              bit[2]=0(digest is LSBF), 1(digest is MSBF)
 * @return 0=success(PKE started), non-zero=error(PKE not started)
 * @notes - Caller must have previously programmed elliptic curve
 * into PKE e.g. pke_ec_prog_curve().
 */
extern uint8_t ec_kcdsa_verify(const uint8_t* q,
                                 uint16_t qlen,
                                 const uint8_t* sig,
                                 uint16_t slen,
                                 const uint8_t* hash,
                                 uint16_t hlen,
                                 uint16_t flags);

extern const ELLIPTIC_CURVE ec_sect571r1;
extern const ELLIPTIC_CURVE ec_sect409r1;
extern const ELLIPTIC_CURVE ec_sect283r1;
extern const ELLIPTIC_CURVE ec_sect233r1;
extern const ELLIPTIC_CURVE ec_sect163r2;
extern const ELLIPTIC_CURVE ec_secp521r1;
extern const ELLIPTIC_CURVE ec_secp384r1;
extern const ELLIPTIC_CURVE ec_secp256r1;
extern const ELLIPTIC_CURVE ec_secp224r1;
extern const ELLIPTIC_CURVE ec_secp192r1;


/*
 * Program PKE->CONFIG OpPtrA b[4:0], OpPtrB b[12:8], and OpPtrC (result) b[20:16]
 * PKE->COMMAND
 *   preserve bits[30:29, 15:8, 7]
 *   program bits[31, 6:0].
 *     Bit[31]=1
 *     Bits[6:4] = 10b (0x2) ECC ops
 *     Bits[3:0] = 0x0 Point Doubling,  params = P1x, P1y
 *               = 0x1 Point Addition,  params = P1x, P1y, P2x, P2y
 *               = 0x2 Point Multiplication, params = k, P1x, P1y
 *               = 0x3 Check AB, no other params
 *               = 0x4 Check n, no other params
 *               = 0x5 Check point coordiantes < p, params = P1x, P1y
 *               = 0x6 Check point on curve, params = P1x, P1y
 *               = 0x7 - 0xF reserved
 */
extern uint8_t ec_point_double(const uint8_t* pxy,
                                 const uint16_t coord_len,
                                 const bool msbf);

/* ec_point_add
 * @param pointer to Operand P1
 * @param pointer to Operand P2
 * @param number of bytes to add
 * @param flag to indicate data in reverse order
 * @return PKE_RET_OK on success, ECC_ERR_BUSY if PKE engine busy, ECC_ERR_BAD_PARAM if
 *         parameters have invalid values
 */
extern uint8_t ec_point_add(const uint8_t* p1xy,
                                   const uint8_t* p2xy,
                                   const uint16_t coord_len,
                                   const bool msbf);

/* ec_point_scalar_mult2
 * @param pointer to Operand P1
 * @param pointer to Operand P2
 * @param pointer to scalar
 * @param number of bytes
 * @param flag to indicate data in reverse order
 * @return PKE_RET_OK on success, ECC_ERR_BUSY if PKE engine busy, ECC_ERR_BAD_PARAM if
 *         parameters have invalid values
 */
extern uint8_t ec_point_scalar_mult2(const uint8_t* px,
                                       const uint8_t* py,
                                       const uint8_t* pscalar,
                                       const uint16_t byte_len,
                                       const bool msbf);

/* ec_point_scalar_mult3
 * @param pointer to Operand P1
 * @param pointer to scalar
 * @param number of bytes
 * @param flag to indicate data in reverse order
 * @return PKE_RET_OK on success, ECC_ERR_BUSY if PKE engine busy, ECC_ERR_BAD_PARAM if
 *         parameters have invalid values
 */
extern uint8_t ec_point_scalar_mult3(const uint8_t* pxy,
                                       const uint8_t* pscalar,
                                       const uint16_t byte_len,
                                       const bool msbf);

/* ec_check_poc2 Check if point is on the curve currently programmed into the PKE
 * using for example pke_ec_prog_curve()
 * @param pointer to pointer to point to check
 * @param pointer to pointer to point to check
 * @param number of bytes
 * @param flag to indicate data in reverse order
 * @return PKE_RET_OK on success, ECC_ERR_BUSY if PKE engine busy, ECC_ERR_BAD_PARAM if
 *         parameters have invalid values
 */
extern uint8_t ec_check_poc2(const uint8_t* px,
                               const uint8_t* py,
                               const uint16_t plen,
                               const bool msbf);

/* pke_ec_check_poc3 Check if point is on the curve currently programmed into the PKE
 * using for example pke_ec_prog_curve()
 * @param pointer to pointer to point to check
 * @param number of bytes
 * @param flag to indicate data in reverse order
 * @return PKE_RET_OK on success, ECC_ERR_BUSY if PKE engine busy, ECC_ERR_BAD_PARAM if
 *         parameters have invalid values
 */
extern uint8_t pke_ec_check_poc3(const uint8_t* p,
                               const uint16_t plen,
                               const bool msbf);

/* ec_check_point_less_prime
 * Check Point coordinates are less than prime
 * Requires curve has been programmed into PKE via
 * pke_ec_prog_curve().
 */
extern uint8_t ec_check_point_less_prime(const uint8_t* pxy,
                                         const uint16_t plen,
                                         const bool msbf);

/*
 * Check EC Curve parameters a & b
 * Requires curve has been programmed into PKE via
 * pke_ec_prog_curve().
 */
extern uint8_t ec_check_ab(void);

/*
 * Check EC Curve order (parameter n)
 * Requires curve has been programmed into PKE via
 * pke_ec_prog_curve().
 */
extern uint8_t ec_check_n(void);

/* pke_modular_math
 * @param op_size b[6:4]=operation, b[7]=Field, b[15:8]=size in
 * units of 64 bits, bit[16]=0(parameters LSBF), 1(parameters MSBF)
 * @param pointer to parameter P
 * @param byte length of P
 * @param pointer to parameter A
 * @param byte length of A
 * @param pointer to parameter B
 * @param byte length of B
 * @notes Field F(p) = 0, F(2^m) = 1, the size in units of 64 bits
 * must be in [0x01, 0x40] or [64-bits, 4096-bits]. Operation in
 * bits[6:4]
 * 0x00 Reserved
 * 0x01 C = (A+B) mod P
 * 0x02 C = (A-B) mod P
 * 0x03 C = (A*B) mod P (P odd)
 * 0x04 C = B mod P (P odd), A is ignored
 * 0x05 C = (A/B) mod P (P odd)
 * 0x06 C = (1/B) mod P (P odd)
 * 0x07 Reserved
 * 0x08 C = (A * B) F(p) only, P is ignored
 * 0x09 C = (1/B) mod P (P even), A is ignored
 * 0x0A C = B mod P (P even), A is ignored
 *
 */
extern uint8_t modular_math(uint32_t op_size,
                              const void *P,
                              uint16_t pnbytes,
                              const void *A,
                              uint16_t anbytes,
                              const void *B,
                              uint16_t bnbytes);

#define ED_PARAM_AX    (0u)
#define ED_PARAM_AY    (1u)
#define ED_PARAM_RX    (2u)
#define ED_PARAM_RY    (3u)
#define ED_PARAM_SIG   (4u)
#define ED_PARAM_HASH  (5u)
#define ED_PARAM_MAX   (6u)

typedef struct {
    uint8_t* params[ED_PARAM_MAX];
    uint16_t paramlen[ED_PARAM_MAX];
    uint16_t flags;
    uint16_t rsvd;
} Ed25519_SIG_VERIFY;

/* ec25519_point_mult
 * @param pointer to parameter Px
 * @param byte length of Px
 * @param pointer to parameter K
 * @param byte length of K
 * @param flag to indicate data in reverse order
 * @return 0(PKE started), Non-zero(bad parameter(s) error)
 */
extern uint8_t ec25519_point_mult(const uint8_t* p1x, uint16_t p1x_len,
                   const uint8_t* k, uint16_t k_len,
                   uint16_t flags);

/* ed25519_xrecover
 * @param pointer to parameter y
 * @param byte length of y
 * @param flag to indicate data in reverse order
 * @return 0(PKE started), Non-zero(bad parameter(s) error)
 */
extern uint8_t ed25519_xrecover(const uint8_t* y, uint16_t ylen, uint16_t flags);


/** ed25519_scalar_mult - Multiply point by a scalar for Elliptic Curve
 * 25519. Part of Ed25519.
 * @param pointer to array of bytes containing a point on curve 25519. First
 * 256 bytes are the x-coordinate, second 256 bytes are y-coordinate.
 * @param pointer to array of bytes containing the scalar.
 * @param scalar byte length <= 256.
 * @param flags bit[0] = 0(point byte array is LSBF), 1(point is MSBF)
 *              bit[1] = 0(scalar byte array is LSBF), 1(MSBF)
 * @return 0(PKE started), Non-zero(bad parameter(s) error)
 * @note When done, result located in SCM at
 * Slot[0xA]=x-coordinate, Slot[0xB]=y-coordinate
 *
 * Ed25519 Scalar Multiplication
 * Slot[0] = p
 * Slot[1] = D2
 * OpPtrA = P
 * OpPtrB = e
 * OpPtrC = C = e.P
 */
extern uint8_t ed25519_scalar_mult(const uint8_t* px, uint16_t pxlen,
                       const uint8_t* py, uint16_t pylen,
                       const uint8_t* e,
                       uint16_t elen,
                       uint16_t flags);

/* ed25519_valid_sig
 * @param pointer to structure Ed25519_SIG_VERIFY
 * @return PKE_RET_OK on success, ECC_ERR_BUSY if PKE engine busy, ECC_ERR_BAD_PARAM if
 *         parameters have invalid values
 */
extern uint8_t ed25519_valid_sig(const Ed25519_SIG_VERIFY* psv);

#ifdef __cplusplus
}
#endif


#endif /* INCLUDE_MEC2016_MEC2016_ROM_API_H_ */
