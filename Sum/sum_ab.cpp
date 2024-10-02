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
    while(true){
        cout << "Enter Second Number \n";
        cin >> num2;
        if (num2>=1 && num2 >= num1){
            break;
        } 
    }
    for(int i=num1; i<= num2; i++){
        sum += i;
        
    }
    if(num1 == num2){
            sum +=num1;
        }
    cout << "Sum of Numbers between " << num1 << " and " << num2 << " is " << sum << "\n";
}