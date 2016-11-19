#include <iostream>
#include <vector>
#include <string>

class HelloWorld {

private:

public:
	std::string print() {
		if (true)
			return SuperLongVariable;
	}
	std::string SuperLongVariable;

};

class Hello : public HelloWorld {

private:

public:
Hello() {}

	void initiate() {
		SuperLongVariable = "Hello";
	}
};

class World : public HelloWorld {
	
	private:

	public:
		World() {}
		void initiate(){
			SuperLongVariable = "World";
	}
};

class SuperHelloWorld {

	public:
	Hello* H;
	World* W;

	SuperHelloWorld() {
		H = new Hello();
		W = new World();
	}

	void initiate() {
	 H->initiate();
	 W->initiate();
	}

	bool print() {
		std::cout << H->print() << " " << W->print() << "\n";
		return true;
	}
};

int main(void) {
	SuperHelloWorld* shw = new SuperHelloWorld();
	shw->initiate();
	shw->print();

	return 0;
}