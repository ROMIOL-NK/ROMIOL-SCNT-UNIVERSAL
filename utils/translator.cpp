#include "translator.h"
using namespace std;
const uchar MD5::padding[64] = { 0x80 };	// 初始化附加填充 1000 0000， 设置最高位为1
const char MD5::hex[16] = { '0', '1', '2', '3','4', '5', '6', '7','8', '9', 'a', 'b','c', 'd', 'e', 'f' };

QString myregex(const QString &qstr){   //正则解析函数
    string ans;
    string str = qstr.toStdString();
    string pattern = "\"(.+?)\"";
    regex e("\\[(.*)\\]");
    regex r(pattern);
    smatch m;
    regex_search(str, m, e);

    string temp = m[1];
    auto res = std::vector<string>(9);

    sregex_iterator pos(temp.cbegin(), temp.cend(), r), end;
    for (; pos != end; ++pos) {
        res.push_back(pos->str(1));
    }
    for (auto temp : res)
        ans = temp;

    return QString::fromStdString(ans);
}

QString unicodeToUtf8(QString str) //unicode转utf8
{
    QString result;
    int index = str.indexOf("\\u");
    qDebug()<<index;
    //判断是否存在为转移字符，实践表明只有英文字母是可以正常显示不需要转码的
    if(index!=-1){
        /* 判断字符串是否全为转义字符，实践表明中文和韩文等全是转移
         * 法语和德语只有个别特殊符号是转义字符
         *  */
        if(index==0){   //全为转义
            while (index != -1){
                QString s1 = str.mid(index + 2, 4);
                result.append(s1.toUShort(0, 16));
                index = str.indexOf("\\u", index+5);
            }
            return result.toUtf8().constData();
        }
        else{   //部分转义
            while (index != -1){
                QString s1 = str.mid(index + 2, 4);
                result=s1.toUShort(0, 16);
                str.replace(index-1,7,result.toUtf8().constData());
                index = str.indexOf("\\u", index+5);
            }
            return str;
        }
    }
    return str;
}
MD5::MD5(const string& str) {
    end_flag = false;
    count[0] = count[1] = 0; // 重置bits个数
    // 初始化链接变量（高位->低位）
    /*
     * 错误的赋值，注意高低位
    reg[0] = 0x01234567;
    reg[1] = 0x89abcdef;
    reg[2] = 0xfedcba98;
    reg[3] = 0x76543210;
    */
    reg[0] = 0x67452301;
    reg[1] = 0xefcdab89;
    reg[2] = 0x98badcfe;
    reg[3] = 0x10325476;
    generate((uchar*)const_cast<char*>(str.c_str()), str.length());
}
//获取摘要
const uchar* MD5::getDigest() {
    if (!end_flag) {
        end_flag = true;
        uchar bits[8];
        ulong _reg[4];		// 旧reg
        ulong _count[2];	// 旧count
        ulong index, padLen;
        memcpy(_reg, reg, 16);		// 复制内存，将_reg内存地址的起始位置开始拷贝16字节到reg起始位置中
        memcpy(_count, count, 8);	// 将原始消息长度以64比特（8字节）复制到count后
        encode(count, bits, 8);
        /* Pad out to 56 mod 64. */
        index = (ulong)((count[0] >> 3) & 0x3f);
        padLen = (index < 56) ? (56 - index) : (120 - index);
        generate(padding, padLen);
        generate(bits, 8);
        encode(reg, digest, 16);
        /* 重新存储reg和count */
        memcpy(reg, _reg, 16);
        memcpy(count, _count, 8);
    }
    return digest;
}
void MD5::generate(const uchar* input, int length) {
    ulong i, index, partLen;
    end_flag = false;
    index = (ulong)((count[0] >> 3) & 0x3f);
    if ((count[0] += ((ulong)length << 3)) < ((ulong)length << 3))
        count[1]++;
    count[1] += ((ulong)length >> 29);
    partLen = 64 - index;
    if (length >= partLen) {
        memcpy(&buffer[index], input, partLen);
        change(buffer);	//分组处理，四轮操作
        for (i = partLen; i + 63 < length; i += 64)
            change(&input[i]);
        index = 0;
    }
    else {
        i = 0;
    }
    // 内存拷贝
    memcpy(&buffer[index], &input[i], length - i);
}
//四轮变换操作
void MD5::change(const uchar block[64]) {
    ulong a = reg[0],
        b = reg[1],
        c = reg[2],
        d = reg[3],
        m[16];

    decode(block, m, 64);	// uchar 转 ulong

    FF(a, b, c, d, m[0], 7, 0xd76aa478);
    FF(d, a, b, c, m[1], 12, 0xe8c7b756);
    FF(c, d, a, b, m[2], 17, 0x242070db);
    FF(b, c, d, a, m[3], 22, 0xc1bdceee);
    FF(a, b, c, d, m[4], 7, 0xf57c0faf);
    FF(d, a, b, c, m[5], 12, 0x4787c62a);
    FF(c, d, a, b, m[6], 17, 0xa8304613);
    FF(b, c, d, a, m[7], 22, 0xfd469501);
    FF(a, b, c, d, m[8], 7, 0x698098d8);
    FF(d, a, b, c, m[9], 12, 0x8b44f7af);
    FF(c, d, a, b, m[10], 17, 0xffff5bb1);
    FF(b, c, d, a, m[11], 22, 0x895cd7be);
    FF(a, b, c, d, m[12], 7, 0x6b901122);
    FF(d, a, b, c, m[13], 12, 0xfd987193);
    FF(c, d, a, b, m[14], 17, 0xa679438e);
    FF(b, c, d, a, m[15], 22, 0x49b40821);

    GG(a, b, c, d, m[1], 5, 0xf61e2562);
    GG(d, a, b, c, m[6], 9, 0xc040b340);
    GG(c, d, a, b, m[11], 14, 0x265e5a51);
    GG(b, c, d, a, m[0], 20, 0xe9b6c7aa);
    GG(a, b, c, d, m[5], 5, 0xd62f105d);
    GG(d, a, b, c, m[10], 9, 0x2441453);
    GG(c, d, a, b, m[15], 14, 0xd8a1e681);
    GG(b, c, d, a, m[4], 20, 0xe7d3fbc8);
    GG(a, b, c, d, m[9], 5, 0x21e1cde6);
    GG(d, a, b, c, m[14], 9, 0xc33707d6);
    GG(c, d, a, b, m[3], 14, 0xf4d50d87);
    GG(b, c, d, a, m[8], 20, 0x455a14ed);
    GG(a, b, c, d, m[13], 5, 0xa9e3e905);
    GG(d, a, b, c, m[2], 9, 0xfcefa3f8);
    GG(c, d, a, b, m[7], 14, 0x676f02d9);
    GG(b, c, d, a, m[12], 20, 0x8d2a4c8a);

    HH(a, b, c, d, m[5], 4, 0xfffa3942);
    HH(d, a, b, c, m[8], 11, 0x8771f681);
    HH(c, d, a, b, m[11], 16, 0x6d9d6122);
    HH(b, c, d, a, m[14], 23, 0xfde5380c);
    HH(a, b, c, d, m[1], 4, 0xa4beea44);
    HH(d, a, b, c, m[4], 11, 0x4bdecfa9);
    HH(c, d, a, b, m[7], 16, 0xf6bb4b60);
    HH(b, c, d, a, m[10], 23, 0xbebfbc70);
    HH(a, b, c, d, m[13], 4, 0x289b7ec6);
    HH(d, a, b, c, m[0], 11, 0xeaa127fa);
    HH(c, d, a, b, m[3], 16, 0xd4ef3085);
    HH(b, c, d, a, m[6], 23, 0x4881d05);
    HH(a, b, c, d, m[9], 4, 0xd9d4d039);
    HH(d, a, b, c, m[12], 11, 0xe6db99e5);
    HH(c, d, a, b, m[15], 16, 0x1fa27cf8);
    HH(b, c, d, a, m[2], 23, 0xc4ac5665);

    II(a, b, c, d, m[0], 6, 0xf4292244);
    II(d, a, b, c, m[7], 10, 0x432aff97);
    II(c, d, a, b, m[14], 15, 0xab9423a7);
    II(b, c, d, a, m[5], 21, 0xfc93a039);
    II(a, b, c, d, m[12], 6, 0x655b59c3);
    II(d, a, b, c, m[3], 10, 0x8f0ccc92);
    II(c, d, a, b, m[10], 15, 0xffeff47d);
    II(b, c, d, a, m[1], 21, 0x85845dd1);
    II(a, b, c, d, m[8], 6, 0x6fa87e4f);
    II(d, a, b, c, m[15], 10, 0xfe2ce6e0);
    II(c, d, a, b, m[6], 15, 0xa3014314);
    II(b, c, d, a, m[13], 21, 0x4e0811a1);
    II(a, b, c, d, m[4], 6, 0xf7537e82);
    II(d, a, b, c, m[11], 10, 0xbd3af235);
    II(c, d, a, b, m[2], 15, 0x2ad7d2bb);
    II(b, c, d, a, m[9], 21, 0xeb86d391);

    reg[0] += a;
    reg[1] += b;
    reg[2] += c;
    reg[3] += d;
}
//ulong 转 uchar
void MD5::encode(const ulong* input, uchar* output, int length) {
    for (int i = 0, j = 0; j < length; i++, j += 4) {
        output[j] = (uchar)(input[i] & 0xff);
        output[j + 1] = (uchar)((input[i] >> 8) & 0xff);
        output[j + 2] = (uchar)((input[i] >> 16) & 0xff);
        output[j + 3] = (uchar)((input[i] >> 24) & 0xff);
    }
}
//uchar 转 ulong
void MD5::decode(const uchar* input, ulong* output, int length) {

    for (int i = 0, j = 0; j < length; i++, j += 4) {
        output[i] = ((ulong)input[j]) | (((ulong)input[j + 1]) << 8) | (((ulong)input[j + 2]) << 16) | (((ulong)input[j + 3]) << 24);
    }
}
string MD5::outstr(int length = 32) {
    const uchar* input = getDigest();
    string str;
    //str.reserve(length << 1);
    for (int i = 0; i < 16; i++) {
        int t = input[i];
        int a = t / 16;
        int b = t % 16;
        str.append(1, hex[a]);
        str.append(1, hex[b]);
    }
    if (16 == length) {
        str = str.substr(9 - 1, 16);
    }
    return str;
}

Translator::Translator(){
    from = "auto";
    to = "zh";
    //制作签名
    myurl = "https://fanyi-api.baidu.com/api/trans/vip/translate?";
    appid = "20221110001445094";            //你的appid
    secret_key = "pAmeveNQmmxqMkAI56gI";    //你的密钥
}

void Translator::SetQstr(QString qstr){
    q = qstr.toStdString();
}
QString Translator::GetUrl(){
    time_t myt = time(NULL);  //获取时间戳
    salt = to_string(myt);
    sign = "";
    //签名拼接
    sign.append(appid);
    sign.append(q);
    sign.append(salt);
    sign.append(secret_key);

    //签名MD5加密
    MD5 md5 = MD5(sign);
    sign = md5.outstr(32).c_str();

    //制作url
    myurl.append("&q=");
    myurl.append(q);
    myurl.append("&from=");
    myurl.append(from);
    myurl.append("&to=");
    myurl.append(to);
    myurl.append("&appid=");
    myurl.append(appid);
    myurl.append("&salt=");
    myurl.append(salt);
    myurl.append("&sign=");
    myurl.append(sign);
    //string转Qstring
    url=QString::fromStdString(myurl);
    return url;
}
