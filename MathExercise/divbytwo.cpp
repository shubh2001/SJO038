#include <iostream>
using namespace std;


bool DivByTwo(int n) {
    while(n>1){
        if(n%2 != 0) {return false;
    }
    n/=2;
    }
    return true;
}

int main (){
    int num;
    cout << "Enter a number \n";
    cin >> num;

    if(DivByTwo(num)){
        cout << num << " is a power of 2. \n";}
    else{
        cout << num << " is not a power of 2. \n";
    }
}