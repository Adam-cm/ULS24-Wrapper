// Copyright 2014-2017, Anitoa Systems, LLC
// All rights reserved

#pragma once

#ifdef _WIN32
#include <windows.h>
#include <tchar.h>
#include <afx.h>
#else
#include <stdint.h>
#include <string>
#include <vector>
typedef uint8_t BYTE;
typedef int BOOL;
#define TRUE 1
#define FALSE 0
typedef char TCHAR;
typedef std::string CString;
#endif

#define TRIM_MAX_WORD 1024
#define TRIM_IMAGER_SIZE 12
#define NUM_EPKT 4
#define EPKT_SZ 64
#define MAX_TRIMBUFF 1024

class CTrimNode {
public:
	CString name;
	double kb[TRIM_IMAGER_SIZE][6];
	int kbi[TRIM_IMAGER_SIZE][6];
	double fpn[2][TRIM_IMAGER_SIZE];
	int fpni[2][TRIM_IMAGER_SIZE];
	double tempcal[TRIM_IMAGER_SIZE];
	BYTE rampgen;
	BYTE range;
	BYTE auto_v20[2];
	BYTE auto_v15;
	BYTE version;
	BYTE trim_buff[MAX_TRIMBUFF];
	int tbuff_size;
	int tbuff_rptr;
	int fp;

	CTrimNode();
	void Initialize();
};

class CTrimReader {
public:
	CTrimReader();
	~CTrimReader();

	// Member variables for file parsing and trim data
#ifdef _WIN32
	CFile InFile;
#else
	// Use std::ifstream for file operations on non-Windows
#endif
	CString WordBuf[TRIM_MAX_WORD];
	CString CurWord;
	int WordIndex;
	int MaxWord;
	bool fileLoaded;

	CTrimNode Node[16];
	CTrimNode* curNode;
	int NumNode;

	BYTE id;
	BYTE version;
	BYTE serial_number1, serial_number2;
	BYTE num_channels, num_wells, num_pages;
	BYTE well_format, channel_format;
	std::vector<BYTE> id_str;

	BYTE trim_buff[MAX_TRIMBUFF];
	int tbuff_rptr;

	int Load(TCHAR* fn);
	int GetWord();
	int Match(CString s);
	void Parse();
	void ParseNode();
	void ParseMatrix();
	void ParseArray(int gain);
	void ParseValue(int gain);
	int ADCCorrection(int NumData, BYTE HighByte, BYTE LowByte, int pixelNum, int PCRNum, int gain_mode, int* flag);
	int ADCCorrectioni(int NumData, BYTE HighByte, BYTE LowByte, int pixelNum, int PCRNum, int gain_mode, int* flag);
	void SetV20(BYTE v20);
	void SetGainMode(int gain);
	void SetV15(BYTE v15);
	void Capture12();
	void Capture12(BYTE chan);
	void Capture24();
	void SetRangeTrim(BYTE range);
	void SetRampgen(BYTE rampgen);
	void SetTXbin(BYTE txbin);
	void SetLEDConfig(BOOL IndvEn, BOOL Chan1, BOOL Chan2, BOOL Chan3, BOOL Chan4);
	void SetIntTime(float int_t);
	void SelSensor(BYTE i);
	int ProcessRowData(int (*adc_data)[24], int gain_mode);
	BYTE TrimBuff2Byte();
	void CopyEepromBuffAndRestore();
	void RestoreFromTrimBuff();
	void OnEEPROMRead();
	void EEPROMRead();
	void ReadTrimData();
	void Convert2Int(int c);
	int Add2TrimBuff(int i, int val);
	int Add2TrimBuff(int i, BYTE val);
	int WriteTrimBuff(int k);
	int TrimBuff2Int(int i);
	BYTE TrimBuff2Byte(int i);
	void RestoreTrimBuff(int k);
	void CopyEepromBuff(int k, int index_start);
};