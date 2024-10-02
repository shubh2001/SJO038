#include <iostream>
using namespace std;

int main(){
    int num1;
    int num2;
    int sum = 0;
    
    while(true){
        cout << "Enter First Number \n";
        cin >> num1;
        if (num1>=1){
            break;
        } 
    }
    cout << "Enter Second number \n";
    cin >> num2;
    for(int i=num1; i<= num2; i++){
        sum += i;
    }
    cout << "Sum of Numbers between " << num1 << " and " << num2 << " is " << sum << "\n";
}