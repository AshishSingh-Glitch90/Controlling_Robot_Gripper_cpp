#include <iostream>
#include <fstream>
#include <istream>
#include <string>
#include <sstream>


// int inputFile(std::string fileName, std::vector< std::vector<double> > *matrix)
// {
//     const std::string& delim = " \t";
//     std::string line;
//     std::string strnum;

//     std::ifstream is(fileName);
//     if (!is)
//     {
//         std::cout<< "error reading file!" << std::endl;
//         return 0;
//     }

//     // clear first
//     matrix->clear();

//     // parse line by line
//     while (getline(is, line))
//     {
//         matrix->push_back(std::vector<double>());

//         for (std::string::const_iterator i = line.begin(); i != line.end(); ++ i)
//         {
//             // If i is not a delim, then append it to strnum
//             if (delim.find(*i) == std::string::npos)
//             {
//                 strnum += *i;
//                 if (i + 1 != line.end()) // If it's the last char, do not continue
//                     continue;
//             }

//             // if strnum is still empty, it means the previous char is also a
//             // delim (several delims appear together). Ignore this char.
//             if (strnum.empty())
//                 continue;

//             // If we reach here, we got a number. Convert it to double.
//             double       number;

//             std::istringstream(strnum) >> number;
//             matrix->back().push_back(number);

//             strnum.clear();
//         }
//     }

//     int ncols, nrows = 0;
//     for (std::vector< std::vector<double> >::const_iterator it = matrix->begin(); it != matrix->end(); ++ it)
//     {
//         nrows++;
//         ncols = 0;
//         for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
//         {
//             ncols++;
//         }
//     }
//     return 0;
// }

// code for color display in the terminal
namespace Color {
    enum Code {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 49
    };
    class Modifier {
        Code code;
    public:
        Modifier(Code pCode) : code(pCode) {}
        friend std::ostream&
        operator<<(std::ostream& os, const Modifier& mod) {
            return os << "\033[" << mod.code << "m";
        }
    };
}

#if defined(__linux__)
  //the following are UBUNTU/LINUX ONLY terminal color codes.
    #define DEFAULT     "\033[0m"
    #define RED         "\033[31m"              /* Red */
    #define GREEN       "\033[32m"              /* Green */
#endif

#define UNUSED(stuff) (void)(stuff);

int sign_fun(float x);

// #endif