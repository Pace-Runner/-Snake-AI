# Snake-AI
Snake AI is a Java-based project where an autonomous agent (MyAgent.java) plays the classic Snake game intelligently. The game runs through a provided SnakeRunner.jar file that launches the simulation.

<img width="504" height="499" alt="image" src="https://github.com/user-attachments/assets/07744954-281e-4a71-b823-5817d1a0c4dd" />


🎮 How to Run

Make sure you have Java 8 or later installed.

📹 Demo

Watch the AI in action on YouTube:
[https://youtu.be/E1q_I_Ss_1U](https://youtu.be/E1q_I_Ss_1U)

How it works:

  Game state handling: The agent receives the current position of the snake, the food, and obstacles on the board each turn.  
  Direction control: The AI calculates which move (up, down, left, right) to take based on its analysis of the board.  
  Collision avoidance: Before moving, it checks for potential collisions with walls or its own body.  
  Food targeting: It identifies the nearest food and attempts to navigate toward it efficiently.  
  Fallback logic: If the direct path to food is blocked, the agent safely reroutes to prevent getting trapped.  
