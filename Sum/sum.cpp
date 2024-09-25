#include <iostream>
#include <string>
using namespace std;

int main()
{
    int usernum;
    cout << "Enter a number greater than 1\n";
    cin >> usernum;


    int sum = 0;
    for (int i = 1; i <= usernum; ++i){
        sum += i;
    }

    cout << "Sum of integers from 1 to " << usernum << " is: " <<  sum << "\n";
    return 0;
}