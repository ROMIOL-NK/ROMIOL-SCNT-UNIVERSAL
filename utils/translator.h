#ifndef TANSLATOR_H
#define TANSLATOR_H

#include <iostream>
#include <QString>
#include <QDebug>
#include <QObject>
#include <time.h>
#include <regex>


typedef unsigned char uchar;
typedef unsigned long ulong;

//步函数
#define F(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & (~z)))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~z)))
//循环左移
#define ROL(x, n) (((x) << (n)) | ((x) >> (32-(n))))
//  四轮操作
#define FF(a, b, c, d, mj, s, ti) { (a) += F ((b), (c), (d)) + (mj) + ti; (a) = ROL ((a), (s)); (a) += (b);}
#define GG(a, b, c, d, mj, s, ti) { (a) += G ((b), (c), (d)) + (mj) + ti; (a) = ROL ((a), (s)); (a) += (b);}
#define HH(a, b, c, d, mj, s, ti) { (a) += H ((b), (c), (d)) + (mj) + ti; (a) = ROL ((a), (s)); (a) += (b);}
#define II(a, b, c, d, mj, s, ti) { (a) += I ((b), (c), (d)) + (mj) + ti; (a) = ROL ((a), (s)); (a) += (b);}

QString myregex(const QString &qstr);
QString unicodeToUtf8(QString str);

class MD5 {
public:
    MD5(const std::string& str);
    const uchar* getDigest();	// 获取生成的摘要
    std::string outstr(int);			// 返回字符串
private:
    void generate(const uchar* input, int length);
    void change(const uchar block[64]);							// 四轮操作
    void encode(const ulong* input, uchar* output, int length);	// uchar to ulong
    void decode(const uchar* input, ulong* output, int length); // ulong to uchar

    ulong reg[4];		// ABCD
    ulong count[2];	    // 长度扩充
    uchar buffer[64];	// 输入buffer
    uchar digest[16];	// 生成的摘要
    bool end_flag;		// 结束标志
    static const uchar padding[64];
    static const char hex[16];
};

class Translator: public QObject
{
    Q_OBJECT
private:
    /* 先生成string类型url再转成Qstring*/
    std::string myurl;       //存放string类型url
    std::string appid;       //appid
    std::string q;           //string类型翻译的内容
    std::string from;        //翻译内容的语种，一般为‘auto’
    std::string to;          //翻译目标语种
    std::string salt;        //一串随机数，我是用时间戳当成随机数
    std::string secret_key;  //密钥
    std::string sign;        //MD5加密的签名
    QString url;        //Qstring类型url

public:
    Translator();
    QString GetUrl();       //生成 Url
    void SetQstr(QString);  //设置翻译内容
};

#endif // TANSLATOR_H
