#include <iostream>
#include <string>
using namespace std;

int main()
{
    string myname;
    cout << "What is your name? .\n";
    getline (cin, myname);
    cout << "Bon Dia " << myname << ".\n";
    return 0;

}