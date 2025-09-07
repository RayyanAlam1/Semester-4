#include<iostream>
#include<fstream>
#include<cstring>
#include<cstdlib>
#include<algorithm>

using namespace std;

class User {
private:
    string username;
    string password;

public:
	
	User(){
	}
    User(const string& _username, const string& _password) : username(_username), password(_password) {}

    string getUsername() const {
        return username;
    }

    bool checkPassword(const string& _password) const {
        return password == _password;
    }
};

const int MAX_USERS = 100; 
User users[MAX_USERS]; 
int numUsers = 0; 

void signUp() {
    if (numUsers >= MAX_USERS) {
        cout << "Cannot sign up. Maximum number of users reached." << endl;
        return;
    }
    string username, password;
    cout << "Enter username: ";
    cin >> username;
    cout << "Enter password: ";
    cin >> password;
    users[numUsers++] = User(username, password);
    cout << "User successfully signed up!" << endl;
}

bool login() {
    string username, password;
    cout << "Enter username: ";
    cin >> username;
    cout << "Enter password: ";
    cin >> password;
    for (int i = 0; i < numUsers; ++i) {
        if (users[i].getUsername() == username && users[i].checkPassword(password)) {
            return true;
        }
    }
    cout << "Login failed. Incorrect username or password." << endl;
    return false;
}

class SimpleRegex {
private:
    string pattern;

public:
    SimpleRegex(const string& pattern) : pattern(pattern) {}

    bool match(const string& text) {
        size_t found = text.find(pattern);
        return found != string::npos;
    }
};

void loadDictionary(const string& dictionaryFile, string dictionary[], int& size) {
    ifstream file(dictionaryFile);
    if (file.is_open()) {
        string word;
        while (file >> word) {
            for (char& c : word) {
                c = tolower(c);
            }
            dictionary[size++] = word;
        }
        file.close();
    } else {
        cerr << "Unable to open dictionary file." << endl;
    }
}

void toLower(string& data) {
    for (char& c : data) {
        c = tolower(c);
    }
}


bool isWordInDictionary(const string& word, const string dictionary[], int size) {
    for (int i = 0; i < size; ++i) {
        if (dictionary[i] == word) {
            return true;
        }
    }
    return false;
}

void checkSpelling(const string& text, const string dictionary[], int size) {
    SimpleRegex wordRegex("[a-zA-Z]+");
    string word;
    for (char ch : text) {
        if (isalpha(ch)) {
            word += ch;
        } else {
            if (!word.empty() && !wordRegex.match(word)) {
                toLower(word);  
                bool found = isWordInDictionary(word, dictionary, size);
                if (!found) {
                    cout << "Spelling error: " << word << endl;
                }
                word.clear();
            }
        }
        
    }
    
        if (!word.empty() && !wordRegex.match(word)) {
        toLower(word);
        if (!isWordInDictionary(word, dictionary, size)) {
            cout << "Spelling error: " << word << endl;
        }
    }


}

string findMostSimilarWord(const string& misspelled, const string dictionary[], int size) {
    string bestMatch = misspelled;
    int maxMatchingChars = 0;
    int bestMatchSizeDifference = misspelled.length(); 

    for (int i = 0; i < size; ++i) {
        const string& word = dictionary[i];
        int matchingChars = 0;
        for (size_t j = 0; j < min(word.length(), misspelled.length()); ++j) {
            if (word[j] == misspelled[j]) {
                matchingChars++;
            }
        }
        if (matchingChars > maxMatchingChars || 
            (matchingChars == maxMatchingChars && abs(static_cast<int>(word.length()) - static_cast<int>(misspelled.length())) < bestMatchSizeDifference)) {
            maxMatchingChars = matchingChars;
            bestMatch = word;
            bestMatchSizeDifference = abs(static_cast<int>(word.length()) - static_cast<int>(misspelled.length()));
        }
    }

    return bestMatch;
}


string correctSpelling(const string& text, const string dictionary[], int size) {
    string correctedText, word;

    for (char ch : text) {
        if (isalpha(ch)) {
            word += tolower(ch);
        } else {
            if (!word.empty()) {
                string correctedWord = findMostSimilarWord(word, dictionary, size);
                correctedText += correctedWord;
                word.clear();
            }
            correctedText += ch;
        }
    }

    if (!word.empty()) {
        string correctedWord = findMostSimilarWord(word, dictionary, size);
        correctedText += correctedWord;
    }
    
    return correctedText;
}



int main(){
	
    bool isLoggedIn = false;

	system("cls");
    while (!isLoggedIn) {
        int choice;
        cout << "\nChoose an option:" << endl;
        cout << "1. Sign Up" << endl;
        cout << "2. Login" << endl;
        cout << "3. Exit" << endl;
        cout << "Enter your choice: ";
        cin >> choice;

        switch (choice) {
            case 1:
                signUp();
                break;
            case 2:
                isLoggedIn = login(); 
                break;
            case 3:
                return 0; 
            default:
                cout << "Invalid choice. Please try again." << endl;
        }
    }
    
    here:
    
//    system("cls");
    
    cout<<endl;
    cout<<"============================--Welcome To Regular Expression Based Spell Checker--======================================="<<endl;
	cout<<endl<<endl<<endl;
    const int MAX_WORDS = 50000;
    string dictionary[MAX_WORDS];
    int dictionarySize = 0;
    loadDictionary("dictionary.txt", dictionary, dictionarySize);

    string text;
    cout<<"Enter A Text:"<<endl;
    fflush(stdin);
    getline(cin,text);
    string correctedText = correctSpelling(text, dictionary, dictionarySize);

    cout << "Original text: " << text << endl;
	cout<<endl; 
    checkSpelling(text, dictionary, dictionarySize);
    cout<<endl; 
    cout << "Corrected text: " << correctedText << endl;
	
	sleep(2);
	
	goto here;
	
    return 0;
}
