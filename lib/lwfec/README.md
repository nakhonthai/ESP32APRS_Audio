# LwFEC - Lightweight Forward Error Correction library
LwFEC is a full Reed-Solomon FEC encoder and decoder C library dedicated for embedded and other RAM usage restricted systems.
## Memory usage
 The aim of this library is to be safe and deterministic, thus:
* No heap (malloc) is used
* No stack-allocated arrays are used

Parity byte count limit can be changed freely according to application requirements. This allows the memory usage to stay at the required minimum.

## Example usage
The example below shows how to initialize library, encode and decode a message:
```C
#include <stdint.h> //standard integer types header
#include "rs.h" //RS FEC library

#define K 12 //data size
#define T 5 //parity size
#define FCR 0 //first consecutive root index

struct LwFecRS rs; //RS encoder/decoder instance

void LwFEC(void)
{
    RsInit(&rs, T, FCR);
    uint8_t data[RS_BLOCK_SIZE] = {'H', 'e', 'l', 'l', 'o', ' ', 'w', 'o', 'r', 'l', 'd', '\0'};
    RsEncode(&rs, data, N); //encode message

    //then N=RS_BLOCK_SIZE bytes of data are sent through some channel
    //or stored on some disk
    //this may introduce errors that the decoder will try to fix
    
    //alter some bytes for demonstration
    data[1] = 'h';
    data[5] = 'x';

    uint8_t bytesFixed = 0; //store number of corrected bytes here
    if(RsDecode(&rs, data, K, &bytesFixed)) //decode message
    {
        //message decoded succesfully
    }
    else
    {
        //too many errors in message
    }
}
```
## Important notes
The block size is always equal to RS_BLOCK_SIZE (255 bytes) due to use of GF(2^8), however the library can handle any data size *K* provided *T + K <= RS_BLOCK_SIZE*. The remaining block space is padded automatically with zeros. The application just needs to provide *K* bytes of data.
## Size limit change
Size limits used for array preallocation are stored in *rs.h*:
```C
#define RS_MAX_REDUNDANCY_BYTES 64 //maximum parity bytes
```
## License
The code is based on [*Reed-Solomon codes for coders*](https://en.wikiversity.org/wiki/Reed%E2%80%93Solomon_codes_for_coders).
The project is licensed under the GNU GPL v3 license (see [LICENSE](LICENSE)).

