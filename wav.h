#include <File_Config.h>


//������� ��������� ��� ���� �����
typedef struct {
	U32	 Riff; // - ������������� ����� = "RIFF" = 0x46464952
	U32  Len;  // - ����� ����� ��� ����� ���������
} IDRiff;


typedef struct {
        //��������� WAV
	U32 Wav; // - ������������� = "WAVE" = 0x45564157
	U32 Fmt; // - ������������� = "fmt " = 0x20746D66
	U32 Len; // - ����� ����� ����� WAV - �����,
        //�������� ����� WAV
    U16 Type;  	// - ��� �������� ������, ������ - !!!
		    		//	1 - ������ �������;
		    		//	0x101 - IBM mu-law;
                    //  0x102 - IBM a-law;
                    //  0x103 - ADPCM.
    U16 Channels;          //- ����� ������� 1/2 - !!!
    U32 SamplesPerSec;     //- ������� ������� - !!!
    U32 AvgBytesPerSec;    //- ������� ������ ������
    U16 Align;             //- ������������
    U16 Bits;              //- ����� ��� �� �������  - !!!
} IDWave;

typedef struct {
	U32 Data; //- ������������� ="data" =0x61746164
    U32 Len;  // - ����� ������� � ������( ������ 2 )
} IDSampleWave;

typedef struct 
 {
  S16 L;
  S16 R;
 } IDubleChFrame;

typedef struct 
 {
  S16 L;
 } ISingleChFrame;


S8 wav_CreateFile(const char *FileName, void *Frame, U32 LengthFrame, U8 Channels, U32 SamplesPerSec);
S8 dat_CreateFile(const char *FileName, void *Frame, U32 LengthFrame, float *k_norm); //�������� ��� ������
void dat_CreateFile_edit(char *FileName, void *Frame, U32 LengthFrame, float *k_norm); //�������� ��� ������

#define wav_WriteMonoFile(a,b,c,d) wav_CreateFile(a,b,c,1,d); 
#define wav_WriteSterFile(a,b,c,d) wav_CreateFile(a,b,c,2,d); 


