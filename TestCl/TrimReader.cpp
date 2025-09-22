// Copyright 2014-2017, Anitoa Systems, LLC
// All rights reserved

#include "stdafx.h"
#include "TrimReader.h"


#define SAW_TOOTH2		// Newer Sawtooth algorithm. USe 2 pass low byte correction
#define NON_CONTIGUOUS

extern BYTE TxData[];		// the buffer of sent data to COMX
extern BYTE RxData[];		// the buffer of received data from COMX

extern int chan_num;
extern int Continue_Flag;

BYTE EepromBuff[16 + 4 * NUM_EPKT][EPKT_SZ + 1];		// 16 pages maximum - enough to support 16 well 4 channel.


// Node

CTrimNode::CTrimNode()
{
	Initialize();
}

void
CTrimNode::Initialize()
{
	int i;

	for (i = 0; i < TRIM_IMAGER_SIZE; i++) {
		kb[i][0] = 1;		// k
		kb[i][1] = 0;		// b
		kb[i][2] = 0;		// k2
		kb[i][3] = 0;		// b2
		kb[i][4] = 0;		// c
		kb[i][5] = 0;		// h

		fpn[0][i] = 0;
		fpn[1][i] = 0;

		if (!i) tempcal[i] = 1;
		else tempcal[i] = 0;
	}

	rampgen = 0x88;
	range = 0xf;

	auto_v20[0] = 0x8;
	auto_v20[1] = 0xa;

	auto_v15 = 0x8;
	version = 0x0;

	tbuff_size = 0;
	tbuff_rptr = 0;
}

// Reader

CTrimReader::CTrimReader()
{
	//	InFile = 0; 
	curNode = NULL;
	NumNode = 0;

	WordIndex = 0;

	fileLoaded = false;
}

CTrimReader::~CTrimReader()
{
	if (fileLoaded) InFile.Close();
}

int CTrimReader::Load(TCHAR* fn)
{
	int e;
	CString FileBuf;

	e = InFile.Open(fn, CFile::modeRead);

	fileLoaded = e;

	if (!e) return e;

	DWORD fl = InFile.GetLength();

	char* buf = new char[fl];

	InFile.Read(buf, fl);

	FileBuf = buf;

	delete buf;

	int ep;

	CString delimit = CString(", \t\r\n");

	int i = 0;

	FileBuf.TrimLeft(delimit);

	while (((ep = FileBuf.FindOneOf(delimit)) != -1) && i < TRIM_MAX_WORD)
	{
		WordBuf[i] = FileBuf.Mid(0, ep);
		int l = FileBuf.GetLength();
		FileBuf = FileBuf.Mid(ep, (l - ep));
		FileBuf.TrimLeft(delimit);
		i++;
	}

	MaxWord = i;
	FileBuf.Empty();

	return e;
}

int CTrimReader::GetWord()
{
	CurWord = WordBuf[WordIndex];
	WordIndex++;

	return WordIndex;
}

int CTrimReader::Match(CString s)
{
	return (int)(CurWord.Compare(s) == 0);
}

void CTrimReader::Parse()
{
	CString Name;
	int i = 0;

	if (!InFile)
		return;

	for (;;) {
		if (GetWord() == MaxWord)
			break;

		if (Match(CString("DEF"))) {
			GetWord();
			Name = CurWord;

			GetWord();
			if (Match(CString("{"))) {
				curNode = Node + i;
				i++;
				curNode->name = Name;
				ParseNode();
			}
			else break;
		}
		else break;
	}

	NumNode = i;
}


void CTrimReader::ParseNode()
{
	if (!InFile)
		return;

	for (;;) {
		if (GetWord() == MaxWord)
			break;

		if (Match(CString("Kb"))) {
			GetWord();
			if (Match(CString("{"))) {
				ParseMatrix();

				GetWord();
				if (!Match(CString("}")))
					return;
			}
			else return;
		}
		else if (Match(CString("Fpn_lg"))) {
			GetWord();
			if (Match(CString("{"))) {
				ParseArray(0);

				GetWord();
				if (!Match(CString("}")))
					return;
			}
			else return;
		}
		else if (Match(CString("Fpn_hg"))) {
			GetWord();
			if (Match(CString("{"))) {
				ParseArray(1);

				GetWord();
				if (!Match(CString("}")))
					return;
			}
			else return;
		}
		else if (Match(CString("Temp_calib"))) {
			GetWord();
			if (Match(CString("{"))) {
				ParseArray(2);

				GetWord();
				if (!Match(CString("}")))
					return;
			}
			else return;
		}
		else if (Match(CString("Rampgen"))) {
			GetWord();
			if (Match(CString("{"))) {
				ParseValue(2);

				GetWord();
				if (!Match(CString("}")))
					return;
			}
			else return;
		}
		else if (Match(CString("AutoV20_lg"))) {
			GetWord();
			if (Match(CString("{"))) {
				ParseValue(0);

				GetWord();
				if (!Match(CString("}")))
					return;
			}
			else return;
		}
		else if (Match(CString("AutoV20_hg"))) {
			GetWord();
			if (Match(CString("{"))) {
				ParseValue(1);

				GetWord();
				if (!Match(CString("}")))
					return;
			}
			else return;
		}
		else if (Match(CString("AutoV15"))) {
			GetWord();
			if (Match(CString("{"))) {
				ParseValue(3);

				GetWord();
				if (!Match(CString("}")))
					return;
			}
			else return;
		}
		else if (Match(CString("}"))) {
			return;
		}
		else
			return;
	}
}

void CTrimReader::ParseMatrix()
{
	for (int i = 0; i < TRIM_IMAGER_SIZE; i++) {
		for (int j = 0; j < 4; j++) {
			if (GetWord() == MaxWord)
				break;
			curNode->kb[i][j] = _tstof((LPCTSTR)CurWord); // atof(CurWord);
		}
	}
}


void CTrimReader::ParseArray(int gain)
{
	for (int i = 0; i < 12; i++) {
		GetWord();
		if (gain == 2)
			curNode->tempcal[i] = _tstof((LPCTSTR)CurWord);
		else
			curNode->fpn[gain][i] = _tstof((LPCTSTR)CurWord);
	}
}

// gain: 0, 1 - auto_v20[0, 1]; 2: rampgen; 3: auto_v15

void CTrimReader::ParseValue(int gain)
{
	GetWord();

	CString word = CurWord;
	word.MakeLower();
	int p = word.Find(CString("0x"));
	int l = word.GetLength();
	word = word.Mid(p + 2, l - p - 2);
	unsigned int val = _tstoi((LPCTSTR)word);

	val = _tcstoul((LPCTSTR)word, 0, 16);

	if (gain == 2)
		curNode->rampgen = val;
	else if (gain == 3)
		curNode->auto_v15 = val;
	else
		curNode->auto_v20[gain] = val;
}

#define DARK_LEVEL 100
#define DARK_MANAGE

// NumData =  "Column Number"
// pixekNum = "Frame Size"

// With the Sawtooth method, we need to gather denser data and perform a better data fitting.

int CTrimReader::ADCCorrection(int NumData, BYTE HighByte, BYTE LowByte, int pixelNum, int PCRNum, int gain_mode, int* flag)
{
	int hb, lb, lbc;
	int hbln, lbp, hbhn;
	bool oflow = false, uflow = false; //  qerr_big=false;

	//	CString strbuf;
	double ioffset = 0;
	int result;

	hb = (int)HighByte;

	int nd = 0;
	if (pixelNum == 12) nd = NumData;
	else nd = NumData >> 1;

	ioffset = Node[PCRNum - 1].kb[nd][0] * (double)hb + Node[PCRNum - 1].kb[nd][1];

#ifdef NON_CONTIGUOUS

	if (hb >= 128) {
		ioffset += Node[PCRNum - 1].kb[nd][3];
	}

#endif

	hbln = hb % 16;		//

	hbhn = hb / 16;		//

#ifdef SAW_TOOTH		

	ioffset += Node[PCRNum - 1].kb[nd][2] * (hbln - 7);

#endif

	//		ioffset = Node[PCRNum-1].kb[nd][0]*hb + Node[PCRNum-1].kb[nd][1] + Node[PCRNum-1].kb[nd][2] *(hbln - 7);

	lb = (int)LowByte;

	lbc = lb + (int)ioffset;

#ifdef SAW_TOOTH2							// Use lbc, not hbln to calculate sawtooth correction, as hbln tends to be a little unstable	
	ioffset += Node[PCRNum - 1].kb[nd][2] * ((double)lbc - 127) * (1 - (double)hb / 400) / 16;		// 12/19/2016 modification, shrinking sawtooth.
	lbc = lb + (int)ioffset;					// re-calc lbc, 2 pass algorithm
#endif

	lbp = hbln * 16 + 7;

	if (lbc > 255) lbc = 255;
	else if (lbc < 0) lbc = 0;

	int lbpc = lbp - (int)ioffset;				// lpb - ioffset: low byte predicted from the high byte low nibble BEFORE correction
	int qerr = lbp - lbc;					// if the lbc is correct, this would be the quantization error. If it is too large, maybe lb was the saturated "stuck" version

	if (lbpc > 255 + 20) {					// We allow some correction error, because hbln may have randomly flipped.
		oflow = true; *flag = 1;
	}
	else if (lbpc > 255 && qerr > 28) {		// Again we allow some tolerance because hbln may have drifted, leading to fake error
		oflow = true; *flag = 2;
	}
	else if (lbpc > 191 && qerr > 52) {
		oflow = true; *flag = 3;
	}
	else if (qerr > 96) {
		oflow = true; *flag = 4;
	}
	else if (lbpc < -20) {
		uflow = true; *flag = 5;
	}
	else if (lbpc < 0 && qerr < -28) {
		uflow = true; *flag = 6;
	}
	else if (lbpc < 64 && qerr < -52) {
		uflow = true; *flag = 7;
	}
	else if (qerr < -96) {
		uflow = true; *flag = 8;
	}
	else {
		*flag = 0;
	}

	//	if(abs(qerr) > 84) qerr_big = true;

	if (oflow || uflow) {
		result = hb * 16 + 7;
	}
	else {
		result = hbhn * 256 + lbc;
	}

#ifdef DARK_MANAGE

	if (!gain_mode)
		result += -(int)(Node[PCRNum - 1].fpn[1][nd]) + DARK_LEVEL;		// high gain
	else
		result += -(int)(Node[PCRNum - 1].fpn[0][nd]) + DARK_LEVEL;		// low gain

	if (result < 0) result = 0;

#endif

	return result;
}

// This is the integer version of the auto correct function

int CTrimReader::ADCCorrectioni(int NumData, BYTE HighByte, BYTE LowByte, int pixelNum, int PCRNum, int gain_mode, int* flag)
{
	int hb, lb, lbc, hbi;
	int hbln, lbp, hbhn;
	bool oflow = false, uflow = false;

	int ioffset = 0;
	int result;

	int intmax = 32767;
	int intmax256 = 128;

	hb = (int)HighByte;
	hbln = hb % 16;		//
	hbhn = hb / 16;		//

	int nd = 0;
	if (pixelNum == 12) nd = NumData;
	else nd = NumData >> 1;

	int k, b, c, h;

	//	double shrink = 0.022;

	c = (int)(Node[PCRNum - 1].kbi[nd][4]);
	h = (int)(Node[PCRNum - 1].kbi[nd][5]);

	if (hb < 16) {
		k = (int)(Node[PCRNum - 1].kbi[nd][0]);
		b = (int)(Node[PCRNum - 1].kbi[nd][1]) + h / 2;			// 15 is just an empirical value, the first bump is raised higher. To do: what about reverse bump
		c = c + h / 10;
	}
	else if (hb < 128) {
		k = (int)(Node[PCRNum - 1].kbi[nd][0]);
		b = (int)(Node[PCRNum - 1].kbi[nd][1]);					// 
	}
	else {
		k = (int)(Node[PCRNum - 1].kbi[nd][2]);
		b = (int)(Node[PCRNum - 1].kbi[nd][3]);					// 
	}

	ioffset = k * hb / intmax + b / intmax256;

	lb = (int)LowByte;
	lbc = lb + ioffset;

	if (hb > 128) {
		hbi = 128 + (hb - 128) / 2;
	}
	else {
		hbi = hb;
	}

	// Use lbc, not hbln to calculate sawtooth correction, as hbln tends to be a little jittery	
	ioffset += (lbc - 128) * c * (300 - hbi) / (12 * 300 * intmax256);		// 12/19/2016 modification, shrinking sawtooth.
	lbc = lb + ioffset;												// re-calc lbc, 2 pass algorithm

	if (lbc > 255) lbc = 255;
	else if (lbc < 0) lbc = 0;

	lbp = hbln * 16 + 7;

	int lbpc = lbp - ioffset;				// lpb - ioffset: low byte predicted from the high byte low nibble BEFORE correction
	int qerr = lbp - lbc;					// if the lbc is correct, this would be the quantization error. If it is too large, maybe lb was the saturated "stuck" version

	if (lbpc > 255 + 20) {					// We allow some correction error, because hbln may have randomly flipped.
		oflow = true; *flag = 1;
	}
	else if (lbpc > 255 && qerr > 28) {		// Again we allow some tolerance because hbln may have drifted, leading to fake error
		oflow = true; *flag = 2;
	}
	else if (lbpc > 191 && qerr > 52) {
		oflow = true; *flag = 3;
	}
	else if (qerr > 96) {
		oflow = true; *flag = 4;
	}
	else if (lbpc < -20) {
		uflow = true; *flag = 5;
	}
	else if (lbpc < 0 && qerr < -28) {
		uflow = true; *flag = 6;
	}
	else if (lbpc < 64 && qerr < -52) {
		uflow = true; *flag = 7;
	}
	else if (qerr < -96) {
		uflow = true; *flag = 8;
	}
	else {
		*flag = 0;
	}

	if (oflow || uflow) {
		result = hb * 16 + 7;
	}
	else {
		result = hbhn * 256 + lbc;
	}

	//	if (calib2) return result;

#ifdef DARK_MANAGE

	if (!gain_mode)
		result += -(int)(Node[PCRNum - 1].fpni[1][nd]) + DARK_LEVEL;		// high gain
	else
		result += -(int)(Node[PCRNum - 1].fpni[0][nd]) + DARK_LEVEL;		// low gain

	if (result < 0) result = 0;

#endif

	return result;
}


//========== Protocol Engine=================

void CTrimReader::SetV20(BYTE v20)
{
	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x01;		//command
	TxData[2] = 0x02;		//data length
	TxData[3] = 0x04;		//data type, date edit first byte
	TxData[4] = v20;		//real data, date edit second byte
	TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];		//check sum
	if (TxData[5] == 0x17)
		TxData[5] = 0x18;
	else
		TxData[5] = TxData[5];
	TxData[6] = 0x17;		//back code
	TxData[7] = 0x17;		//back code
}

void CTrimReader::SetGainMode(int gain)
{
	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x01;		//command
	TxData[2] = 0x02;		//data length
	TxData[3] = 0x07;		//data type, date edit first byte
	TxData[4] = gain;		//real data, date edit second byte
	TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];		//check sum
	if (TxData[5] == 0x17)
		TxData[5] = 0x18;
	else
		TxData[5] = TxData[5];
	TxData[6] = 0x17;		//back code
	TxData[7] = 0x17;		//back code
}

void CTrimReader::SetV15(BYTE v15)
{
	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x01;		//command
	TxData[2] = 0x02;		//data length
	TxData[3] = 0x05;		//data type, date edit first byte
	TxData[4] = v15;		//real data, date edit second byte

	TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];		//check sum
	if (TxData[5] == 0x17)
		TxData[5] = 0x18;
	else
		TxData[5] = TxData[5];
	TxData[6] = 0x17;		//back code
	TxData[7] = 0x17;		//back code
}

void CTrimReader::Capture12()
{
	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x02;		//command
	TxData[2] = 0x0C;		//data length
	TxData[3] = 0x02;		//data type, date edit first byte
	TxData[4] = 0xff;		//real data
	TxData[5] = 0x00;		//Ô¤ÁôÎ»
	TxData[6] = 0x00;
	TxData[7] = 0x00;
	TxData[8] = 0x00;
	TxData[9] = 0x00;
	TxData[10] = 0x00;
	TxData[11] = 0x00;
	TxData[12] = 0x00;
	TxData[13] = 0x00;
	TxData[14] = 0x00;
	TxData[15] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5] + TxData[6] + TxData[7] + TxData[8] + TxData[9]
		+ TxData[10] + TxData[11] + TxData[12] + TxData[13] + TxData[14];		//check sum
	if (TxData[15] == 0x17)
		TxData[15] = 0x18;
	else
		TxData[15] = TxData[15];
	TxData[16] = 0x17;		//back code
	TxData[17] = 0x17;		//back code
}

void CTrimReader::Capture12(BYTE chan)
{
	if (chan < 1 || chan > 4)
		return;

	chan -= 1;

	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x02;		//command
	TxData[2] = 0x0C;		//data length
	TxData[3] = (chan << 4) | 0x02;		//data type, date edit first byte
	TxData[4] = 0xff;		//real data
	TxData[5] = 0x00;		//
	TxData[6] = 0x00;
	TxData[7] = 0x00;
	TxData[8] = 0x00;
	TxData[9] = 0x00;
	TxData[10] = 0x00;
	TxData[11] = 0x00;
	TxData[12] = 0x00;
	TxData[13] = 0x00;
	TxData[14] = 0x00;
	TxData[15] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5] + TxData[6] + TxData[7] + TxData[8] + TxData[9]
		+ TxData[10] + TxData[11] + TxData[12] + TxData[13] + TxData[14];		//check sum
	if (TxData[15] == 0x17)
		TxData[15] = 0x18;
	else
		TxData[15] = TxData[15];
	TxData[16] = 0x17;		//back code
	TxData[17] = 0x17;		//back code
}

void CTrimReader::Capture24()
{
	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x02;		//command
	TxData[2] = 0x0C;		//data length
	TxData[3] = 0x08;		//data type, date edit first byte
	TxData[4] = 0xff;		//real data
	TxData[5] = 0x00;		//Ô¤ÁôÎ»
	TxData[6] = 0x00;
	TxData[7] = 0x00;
	TxData[8] = 0x00;
	TxData[9] = 0x00;
	TxData[10] = 0x00;
	TxData[11] = 0x00;
	TxData[12] = 0x00;
	TxData[13] = 0x00;
	TxData[14] = 0x00;
	TxData[15] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5] + TxData[6] + TxData[7] + TxData[8] + TxData[9]
		+ TxData[10] + TxData[11] + TxData[12] + TxData[13] + TxData[14];		//check sum
	if (TxData[15] == 0x17)
		TxData[15] = 0x18;
	else
		TxData[15] = TxData[15];
	TxData[16] = 0x17;		//back code
	TxData[17] = 0x17;		//back code
}

void  CTrimReader::SetRangeTrim(BYTE range)
{
	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x01;		//command
	TxData[2] = 0x02;		//data length
	TxData[3] = 0x02;		//data type, date edit first byte
	TxData[4] = range;	//real data, date edit second byte
	//0x01 means send vedio data
	//0x00 means stop vedio data
	TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];		//check sum
	if (TxData[5] == 0x17)
		TxData[5] = 0x18;
	else
		TxData[5] = TxData[5];
	TxData[6] = 0x17;		//back code
	TxData[7] = 0x17;		//back code
}

void  CTrimReader::SetRampgen(BYTE rampgen)
{
	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x01;		//command
	TxData[2] = 0x02;		//data length
	TxData[3] = 0x01;		//data type, date edit first byte
	TxData[4] = rampgen;	//real data, date edit second byte
	//0x01 means send video data
	//0x00 means stop video data
	TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];		//check sum
	if (TxData[5] == 0x17)
		TxData[5] = 0x18;
	else
		TxData[5] = TxData[5];
	TxData[6] = 0x17;		//back code
	TxData[7] = 0x17;		//back code
}

void  CTrimReader::SetTXbin(BYTE txbin)
{
	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x01;		//command
	TxData[2] = 0x02;		//data length
	TxData[3] = 0x08;		//data type, date edit first byte
	TxData[4] = txbin;	//real data, date edit second byte
	//0x01 means send vedio data
	//0x00 means stop vedio data
	TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];		//check sum
	if (TxData[5] == 0x17)
		TxData[5] = 0x18;
	else
		TxData[5] = TxData[5];
	TxData[6] = 0x17;		//back code
	TxData[7] = 0x17;		//back code
}

void CTrimReader::SetLEDConfig(BOOL IndvEn, BOOL Chan1, BOOL Chan2, BOOL Chan3, BOOL Chan4)
{
	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x01;		//command
	TxData[2] = 0x02;		//data length
	TxData[3] = 0x23;		//data type, date edit first byte

	if (!IndvEn)
	{
		TxData[4] = Chan1 ? 1 : 0;									//real data, date edit second byte
	}
	else
	{
		TxData[4] = 0x80;
		if (Chan1)
			TxData[4] |= 1;
		if (Chan2)
			TxData[4] |= 2;
		if (Chan3)
			TxData[4] |= 4;
		if (Chan4)
			TxData[4] |= 8;
	}

	TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];		//check sum

	if (TxData[5] == 0x17)
		TxData[5] = 0x18;
	else
		TxData[5] = TxData[5];

	TxData[6] = 0x17;		//back code
	TxData[7] = 0x17;		//back code
}

void  CTrimReader::SetIntTime(float int_t)
{
	unsigned char* hData = (unsigned char*)&int_t;	//

	BYTE TrimBuf[8];

	TrimBuf[0] = hData[0];	//buffer
	TrimBuf[1] = hData[1];
	TrimBuf[2] = hData[2];
	TrimBuf[3] = hData[3];

	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x01;		//command
	TxData[2] = 0x05;		//data length
	TxData[3] = 0x20;		//data type, date edit first byte
	TxData[4] = TrimBuf[0];		//real data, date edit second byte
	TxData[5] = TrimBuf[1];
	TxData[6] = TrimBuf[2];
	TxData[7] = TrimBuf[3];
	//0x01 means send vedio data
	//0x00 means stop vedio data
	TxData[8] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5] + TxData[6] + TxData[7];		//check sum
	if (TxData[8] == 0x17)
		TxData[8] = 0x18;
	else
		TxData[5] = TxData[5];
	TxData[9] = 0x17;		//back code
	TxData[10] = 0x17;		//back code
}

void CTrimReader::SelSensor(BYTE i)
{
	// TODO: Add your control notification handler code here

	if (i < 1 || i > 4) return;

	TxData[0] = 0xaa;		//preamble code
	TxData[1] = 0x01;		//command
	TxData[2] = 0x03;		//data length
	TxData[3] = 0x26;		//data type
	TxData[4] = i - 1;		//real data
	TxData[5] = 0x00;
	TxData[6] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5];		//check sum
	if (TxData[6] == 0x17)
		TxData[6] = 0x18;
	else
		TxData[6] = TxData[6];
	TxData[7] = 0x17;		//back code
	TxData[8] = 0x17;		//back code
}

#define dppage12 0x02		// display one page with 12 pixel
#define dppage24 0x08		// display one page with 24 pixel

int CTrimReader::ProcessRowData(int (*adc_data)[24], int gain_mode)
{
	static bool rows_received[24] = { 0 };
	static int rows_expected = 12; // or 24 for 24x24

	int result;
	int flag;
	int FrameSize = 0;

	BYTE type = RxData[3];

	switch (type)
	{
	case dppage12:
		FrameSize = 0;
		rows_expected = 12;
		break;
	case dppage24:
		FrameSize = 1;
		rows_expected = 24;
		break;
	default:
		return FrameSize; // Early exit if type is not recognized
	}

	unsigned int rn = RxData[4];

	// Bounds check for row
	if (rn >= (unsigned int)rows_expected)
		return FrameSize;

	// Only process 12 columns for 12-column data
	int ncol = (FrameSize == 0) ? 12 : 24;
	int pixel_offset = 5; // instead of 8

	for (int i = 0; i < ncol; i++) {
		BYTE low = RxData[pixel_offset + i * 2];
		BYTE high = RxData[pixel_offset + i * 2 + 1];
		result = ADCCorrectioni(i, high, low, ncol, chan_num, gain_mode, &flag);
		adc_data[rn][i] = result;
	}

	rows_received[rn] = true;

	// Check if all rows have been received
	bool all_received = true;
	for (int i = 0; i < rows_expected; ++i) {
		if (!rows_received[i]) {
			all_received = false;
			break;
		}
	}
	if (all_received) {
		Continue_Flag = false;
		// Reset for next frame
		memset(rows_received, 0, sizeof(rows_received));
	}

	return FrameSize;
}

BYTE CTrimReader::TrimBuff2Byte()
{
	BYTE r;

	r = trim_buff[tbuff_rptr];
	tbuff_rptr++;

	return r;
}

void CTrimReader::CopyEepromBuffAndRestore()
{
	for (int j = 0; j < EPKT_SZ; j++) {			// parity not copied
		trim_buff[j] = EepromBuff[0][j];		// copy first page
	}

	RestoreFromTrimBuff();

	for (int i = 1; i < num_pages; i++) {
		for (int j = 0; j < EPKT_SZ; j++) {			// parity not copied
			trim_buff[i * EPKT_SZ + j] = EepromBuff[i][j];
		}
	}
}

void CTrimReader::RestoreFromTrimBuff()
{
	// restore trimbuff header first

	tbuff_rptr = 0;				// initialize read pointer

	id = TrimBuff2Byte();

	if (id != 0xa5) {
		serial_number1 = TrimBuff2Byte();
		serial_number2 = TrimBuff2Byte();
		num_channels = TrimBuff2Byte();
		num_wells = TrimBuff2Byte();
		num_pages = TrimBuff2Byte();
	}
	else {
		version = TrimBuff2Byte();
		num_pages = TrimBuff2Byte();

		id_str.clear();
		for (int i = 0; i < 32; i++) {
			id_str.push_back(TrimBuff2Byte());

			//	TrimBuff2Byte();
		}

		serial_number1 = TrimBuff2Byte();
		serial_number2 = TrimBuff2Byte();

		num_channels = TrimBuff2Byte();
		num_wells = TrimBuff2Byte();

		well_format = TrimBuff2Byte();

		channel_format = TrimBuff2Byte();	// 

		// TrimBuff2Byte();
	}
}


extern BOOL ee_continue;

void CTrimReader::OnEEPROMRead()
{
	// EEPROM data, check parity here too.

	BYTE eeprom_parity = 0;
	int index = RxData[7];		// For command type 2d EEPROM read command
	int npages = RxData[6];
	bool parity_ok = true;

	for (int i = 0; i < EPKT_SZ + 1; i++) {		// 
		EepromBuff[index][i] = RxData[8 + i];

		if (i < EPKT_SZ) {
			eeprom_parity += RxData[8 + i];
		}
		else {
			if (eeprom_parity != RxData[8 + i]) {
				//					MessageBox(_T("Packet parity error!"));
				parity_ok = false;
			}
		}
	}


	if (index < npages - 1)
		ee_continue = true;
	else
		ee_continue = false;



}


void CTrimReader::EEPROMRead()
{
	TxData[0] = 0xaa;					//preamble code
	TxData[1] = 0x04;					//command
	TxData[2] = 0x02;					//data length
	TxData[3] = 0x2d;					//data type
	TxData[4] = (BYTE)0x0;			//	real data
	//	TxData[5] = 0x00;
	TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];		//check sum
	if (TxData[5] == 0x17)
		TxData[5] = 0x18;
	else
		TxData[5] = TxData[5];
	TxData[6] = 0x17;		//back code
	TxData[7] = 0x17;		//back code
}

void CTrimReader::ReadTrimData()
{

	CopyEepromBuffAndRestore();			// Compare with with g_DPReader.

	int nchannels = num_channels;
	int npages = num_pages;


	NumNode = nchannels;

	for (int i = 0; i < nchannels; i++) {
		CopyEepromBuff(i, npages + i * NUM_EPKT);
		RestoreTrimBuff(i);
		Node[i].version = 3;				// So it will use integer version KB matrix and FPN values
	}
}

// EEProm buffer related stuff

void CTrimReader::Convert2Int(int c)
{
	int intmax = 32767;
	int intmax256 = 128;

	int i;
	curNode = &Node[c];

	for (i = 0; i < 12; i++) {
		curNode->kbi[i][0] = (int)round(curNode->kb[i][0] * (double)intmax);
		curNode->kbi[i][1] = (int)round(curNode->kb[i][1] * (double)intmax256);
		curNode->kbi[i][2] = (int)round(curNode->kb[i][2] * (double)intmax);
		curNode->kbi[i][3] = (int)round(curNode->kb[i][3] * (double)intmax256);
		curNode->kbi[i][4] = (int)round(curNode->kb[i][4] * (double)intmax256);
		curNode->kbi[i][5] = (int)round(curNode->kb[i][5] * (double)intmax256);
	}

	for (i = 0; i < 12; i++) {
		curNode->fpni[0][i] = (int)round(curNode->fpn[0][i]);
		curNode->fpni[1][i] = (int)round(curNode->fpn[1][i]);
	}
}

int  CTrimReader::Add2TrimBuff(int i, int val)
{
	int k = Node[i].tbuff_size;
	if (k >= MAX_TRIMBUFF - 1) return -1;

	Node[i].trim_buff[k] = val >> 8;					//
	Node[i].trim_buff[k + 1] = val;				//
	Node[i].tbuff_size = k + 2;

	return k + 2;
}

int  CTrimReader::Add2TrimBuff(int i, BYTE val)
{
	int k = Node[i].tbuff_size;
	if (k >= MAX_TRIMBUFF) return -1;

	Node[i].trim_buff[k] = val;							//
	Node[i].tbuff_size = k + 1;

	return k + 1;
}

int  CTrimReader::WriteTrimBuff(int k)
{
	int i, j;
	Node[k].tbuff_size = 0;									// initialize write pointer

#ifdef CALIB_PROGRAM

	int sn = _tstoi(g_ChipID);

	BYTE b0 = (BYTE)sn;
	BYTE b1 = sn >> 8;

	Add2TrimBuff(k, (BYTE)b0);
	Add2TrimBuff(k, (BYTE)b1);
	Add2TrimBuff(k, (BYTE)('O'));							// O for ONT, Oxford Nanopore Tech
#else

	Add2TrimBuff(k, (BYTE)Node[k].name[0]);
	Add2TrimBuff(k, (BYTE)Node[k].name[1]);
	Add2TrimBuff(k, (BYTE)Node[k].name[2]);

#endif

	for (i = 0; i < TRIM_IMAGER_SIZE; i++) {
		for (j = 0; j < 6; j++) {
			Add2TrimBuff(k, (int)Node[k].kbi[i][j]);		// kb
		}
	}

	for (i = 0; i < TRIM_IMAGER_SIZE; i++) {
		Add2TrimBuff(k, (int)(Node[k].fpni[0][i]));
		Add2TrimBuff(k, (int)(Node[k].fpni[1][i]));
	}

	Add2TrimBuff(k, (BYTE)(Node[k].rampgen));
	Add2TrimBuff(k, (BYTE)(Node[k].range));
	Add2TrimBuff(k, (BYTE)(Node[k].auto_v20[0]));
	Add2TrimBuff(k, (BYTE)(Node[k].auto_v20[1]));
	Add2TrimBuff(k, (BYTE)(Node[k].auto_v15));

	Add2TrimBuff(k, (int)round(29.5 * 128));				// tempcal1

	float tcal2 = 0;

#ifdef CALIB_PROGRAM

	tcal2 = (float)(g_curTemp - 29.5 * g_juncTemp);

#endif

	int r = Add2TrimBuff(k, (int)round(tcal2 * 128));


	return r;
}

int CTrimReader::TrimBuff2Int(int i)
{
	_int16 r;		// this is necessary to get negative value correctly
	int k = Node[i].tbuff_rptr;

	r = (Node[i].trim_buff[k] << 8) | (Node[i].trim_buff[k + 1]);
	Node[i].tbuff_rptr += 2;

	return (int)r;
}

BYTE CTrimReader::TrimBuff2Byte(int i)
{
	BYTE r;

	r = Node[i].trim_buff[Node[i].tbuff_rptr];
	Node[i].tbuff_rptr++;

	return r;
}

void  CTrimReader::RestoreTrimBuff(int k)
{
	int i, j;
	Node[k].tbuff_rptr = 0;				// initialize read pointer

	BYTE b0, b1, b2;
	//	CString cid;

	b0 = TrimBuff2Byte(k);
	b1 = TrimBuff2Byte(k);
	b2 = TrimBuff2Byte(k);

	int sn = b1 << 8 | b0;

	//	cid.Format("%c%d", b2, sn);

	//	Node[k].name = cid;

	for (i = 0; i < TRIM_IMAGER_SIZE; i++) {
		for (j = 0; j < 6; j++) {
			Node[k].kbi[i][j] = TrimBuff2Int(k);
		}
	}

	for (i = 0; i < TRIM_IMAGER_SIZE; i++) {
		Node[k].fpni[0][i] = TrimBuff2Int(k);
		Node[k].fpni[1][i] = TrimBuff2Int(k);
	}

	Node[k].rampgen = TrimBuff2Byte(k);
	Node[k].range = TrimBuff2Byte(k);
	Node[k].auto_v20[0] = TrimBuff2Byte(k);
	Node[k].auto_v20[1] = TrimBuff2Byte(k);
	Node[k].auto_v15 = TrimBuff2Byte(k);

	// tempcal restore
	Node[k].tempcal[0] = (double)TrimBuff2Int(k) / (double)128;
	Node[k].tempcal[1] = (double)TrimBuff2Int(k) / (double)128;
}

//#ifdef CALIB_PROGRAM
//extern BYTE EepromBuff[][EPKT_SZ + 1];		// +1 for parity
//#endif

void  CTrimReader::CopyEepromBuff(int k, int index_start)
{
	//#ifdef CALIB_PROGRAM
	for (int i = 0; i < NUM_EPKT; i++) {
		for (int j = 0; j < EPKT_SZ; j++) {			// parity not copied
			Node[k].trim_buff[i * EPKT_SZ + j] = EepromBuff[i + index_start][j];
		}
	}
	//#endif
}




