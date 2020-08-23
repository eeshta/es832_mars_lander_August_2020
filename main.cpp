#include "SDL2SoundEffects.h"
#include <iostream>

int main()
{
	SDL2SoundEffects se;
	se.addSoundEffect("C:/Users/Asus/Downloads/ahem_x.wav");
	se.addSoundEffect("C:/Users/Asus/Downloads/applause_y.wav");
	se.addSoundEffect("C:/Users/Asus/Downloads/boing_x.wav");

	int choice = 0;
	while (choice != -1)
	{
		choice = 0;
		std::cout << "Which sound to play? (enter -1 to exit)\n"
			<< "=>:";
		std::cin >> choice;
		se.playSoundEffect(choice);

		std::cin.clear();
	}
	return 0;
}