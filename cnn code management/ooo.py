import copy
import sys
import pygame
import random
import numpy as np
import pygame.time

# --- PIXELS ---
WIDTH = 600
HEIGHT = 600
ROWS = 3
COLS = 3
SQSIZE = WIDTH // COLS
LINE_WIDTH = 15
CIRC_WIDTH = 15
CROSS_WIDTH = 20
RADIUS = SQSIZE // 4
OFFSET = 50

# --- COLORS ---
BG_COLOR = (28, 170, 156)
LINE_COLOR = (23, 145, 135)
CIRC_COLOR = (239, 231, 200)
CROSS_COLOR = (66, 66, 66)

# --- PYGAME SETUP ---
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('TIC TAC TOE AI')
screen.fill(BG_COLOR)

# --- HELPER FUNCTIONS ---
def create_button(text, x, y, width, height, color, hover_color):
    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()

    if x + width > mouse[0] > x and y + height > mouse[1] > y:
        pygame.draw.rect(screen, hover_color, (x, y, width, height))
        if click[0] == 1:
            return True  # Return True if clicked
    else:
        pygame.draw.rect(screen, color, (x, y, width, height))

    font = pygame.font.Font(None, 50)
    text_surface = font.render(text, True, (0, 0, 0))
    text_rect = text_surface.get_rect(center=(x + (width // 2), y + (height // 2)))
    screen.blit(text_surface, text_rect)
    return False

def post_game_screen():
    screen.fill(BG_COLOR)  # Clear the screen to background color

    # Display "Game Over" text
    font = pygame.font.Font(None, 74)
    text = "Game Over"
    text_surface = font.render(text, True, CIRC_COLOR)
    text_rect = text_surface.get_rect(center=(WIDTH // 2, HEIGHT // 3))
    screen.blit(text_surface, text_rect)

    # Loop to wait for button clicks
    while True:
        new_game_clicked = create_button("New Game", WIDTH // 4 - 100, HEIGHT // 2, 200, 50, LINE_COLOR, (150, 200, 200))
        exit_clicked = create_button("Exit", 3 * WIDTH // 4 - 100, HEIGHT // 2, 200, 50, LINE_COLOR, (150, 200, 200))

        pygame.display.flip()  # Update the screen to show buttons

        # Check for events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if new_game_clicked:
                    return "new_game"
                elif exit_clicked:
                    return "exit"

# --- MENU SCREEN ---
def menu_screen():
    menu_running = True
    screen.fill(BG_COLOR)

    font = pygame.font.Font(None, 74)
    title_text = font.render("TIC TAC TOE", True, CIRC_COLOR)
    title_rect = title_text.get_rect(center=(WIDTH // 2, HEIGHT // 4))
    screen.blit(title_text, title_rect)
    pygame.display.flip()

    button_color = LINE_COLOR
    hover_color = (150, 200, 200)
    game_mode = None

    while menu_running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Check if each button is clicked
        if create_button("Player vs Player", WIDTH // 2 - 100, HEIGHT // 2, 200, 50, button_color, hover_color):
            game_mode = 'pvp'
            menu_running = False

        if create_button("Player vs AI", WIDTH // 2 - 100, HEIGHT // 2 + 100, 200, 50, button_color, hover_color):
            game_mode = 'ai'
            menu_running = False

        if create_button("Exit", WIDTH // 2 - 100, HEIGHT // 2 + 200, 200, 50, button_color, hover_color):
            pygame.quit()
            sys.exit()

        pygame.display.flip()

    return game_mode


# --- CLASSES ---
class Board:

    def __init__(self):
        self.squares = np.zeros((ROWS, COLS))  # 3x3 board initialized with zeros
        self.marked_sqrs = 0  # Keeps track of marked squares

    def final_state(self, show=False):
        '''
            @return 0 if there is no win yet
            @return 1 if player 1 wins
            @return 2 if player 2 wins
        '''
        # Vertical wins
        for col in range(COLS):
            if self.squares[0][col] == self.squares[1][col] == self.squares[2][col] != 0:
                if show:
                    color = CIRC_COLOR if self.squares[0][col] == 2 else CROSS_COLOR
                    iPos = (col * SQSIZE + SQSIZE // 2, 20)
                    fPos = (col * SQSIZE + SQSIZE // 2, HEIGHT - 20)
                    pygame.draw.line(screen, color, iPos, fPos, LINE_WIDTH)
                return self.squares[0][col]

        # Horizontal wins
        for row in range(ROWS):
            if self.squares[row][0] == self.squares[row][1] == self.squares[row][2] != 0:
                if show:
                    color = CIRC_COLOR if self.squares[row][0] == 2 else CROSS_COLOR
                    iPos = (20, row * SQSIZE + SQSIZE // 2)
                    fPos = (WIDTH - 20, row * SQSIZE + SQSIZE // 2)
                    pygame.draw.line(screen, color, iPos, fPos, LINE_WIDTH)
                return self.squares[row][0]

        # Descending diagonal win
        if self.squares[0][0] == self.squares[1][1] == self.squares[2][2] != 0:
            if show:
                color = CIRC_COLOR if self.squares[1][1] == 2 else CROSS_COLOR
                iPos = (20, 20)
                fPos = (WIDTH - 20, HEIGHT - 20)
                pygame.draw.line(screen, color, iPos, fPos, CROSS_WIDTH)
            return self.squares[1][1]

        # Ascending diagonal win
        if self.squares[2][0] == self.squares[1][1] == self.squares[0][2] != 0:
            if show:
                color = CIRC_COLOR if self.squares[1][1] == 2 else CROSS_COLOR
                iPos = (20, HEIGHT - 20)
                fPos = (WIDTH - 20, 20)
                pygame.draw.line(screen, color, iPos, fPos, CROSS_WIDTH)
            return self.squares[1][1]

        # No win yet
        return 0

    def mark_sqr(self, row, col, player):
        """Mark a square for the player (1 or 2) and increase the marked squares count."""
        self.squares[row][col] = player
        self.marked_sqrs += 1

    def empty_sqr(self, row, col):
        """Check if a specific square is empty."""
        return self.squares[row][col] == 0

    def get_empty_sqrs(self):
        """Return a list of all empty squares as (row, col) tuples."""
        empty_sqrs = []
        for row in range(ROWS):
            for col in range(COLS):
                if self.empty_sqr(row, col):
                    empty_sqrs.append((row, col))
        return empty_sqrs

    def isfull(self):
        """Check if the board is completely filled with no empty squares."""
        return self.marked_sqrs == 9

class AI:

    def __init__(self, level=1, player=2):
        self.level = level
        self.player = player

    def rnd(self, board):
        empty_sqrs = board.get_empty_sqrs()
        idx = random.randrange(0, len(empty_sqrs))
        return empty_sqrs[idx]

    def minimax(self, board, maximizing):
        case = board.final_state()

        if case == 1:
            return 1, None
        if case == 2:
            return -1, None
        if board.isfull():
            return 0, None

        if maximizing:
            max_eval = -100
            best_move = None
            empty_sqrs = board.get_empty_sqrs()

            for (row, col) in empty_sqrs:
                temp_board = copy.deepcopy(board)
                temp_board.mark_sqr(row, col, 1)
                eval = self.minimax(temp_board, False)[0]
                if eval > max_eval:
                    max_eval = eval
                    best_move = (row, col)

            return max_eval, best_move

        else:
            min_eval = 100
            best_move = None
            empty_sqrs = board.get_empty_sqrs()

            for (row, col) in empty_sqrs:
                temp_board = copy.deepcopy(board)
                temp_board.mark_sqr(row, col, self.player)
                eval = self.minimax(temp_board, True)[0]
                if eval < min_eval:
                    min_eval = eval
                    best_move = (row, col)

            return min_eval, best_move

    def eval(self, main_board):
        if self.level == 0:
            move = self.rnd(main_board)
        else:
            _, move = self.minimax(main_board, False)
        return move

class Game:

    def __init__(self):
        self.board = Board()
        self.ai = AI()
        self.player = 1  # Player 1 starts
        self.gamemode = 'ai'  # Default mode
        self.running = True  # Game is initially running
        self.winner = None  # No winner at the start
        self.show_lines()

    def show_lines(self):
        # Draw the board lines
        screen.fill(BG_COLOR)
        pygame.draw.line(screen, LINE_COLOR, (SQSIZE, 0), (SQSIZE, HEIGHT), LINE_WIDTH)
        pygame.draw.line(screen, LINE_COLOR, (WIDTH - SQSIZE, 0), (WIDTH - SQSIZE, HEIGHT), LINE_WIDTH)
        pygame.draw.line(screen, LINE_COLOR, (0, SQSIZE), (WIDTH, SQSIZE), LINE_WIDTH)
        pygame.draw.line(screen, LINE_COLOR, (0, HEIGHT - SQSIZE), (WIDTH, HEIGHT - SQSIZE), LINE_WIDTH)

    def display_winner(self):
        # Display who won or if it’s a draw
        font = pygame.font.Font(None, 74)
        if self.winner:
            text = f"Player {self.winner} wins!"
        else:
            text = "It's a draw!"
        text_surface = font.render(text, True, CIRC_COLOR)
        text_rect = text_surface.get_rect(center=(WIDTH // 2, HEIGHT // 2))
        screen.blit(text_surface, text_rect)

    def draw_new_game_button(self):
        # Draws a "New Game" button and returns True if clicked
        return create_button("New Game", WIDTH // 2 - 100, HEIGHT // 2 + 100, 200, 50, LINE_COLOR, (150, 200, 200))

    def make_move(self, row, col):
        # Marks the square and toggles turn
        self.board.mark_sqr(row, col, self.player)
        self.draw_fig(row, col)
        self.next_turn()

    def draw_fig(self, row, col):
        # Draws a cross or circle based on the player
        if self.player == 1:
            start_desc = (col * SQSIZE + OFFSET, row * SQSIZE + OFFSET)
            end_desc = (col * SQSIZE + SQSIZE - OFFSET, row * SQSIZE + SQSIZE - OFFSET)
            pygame.draw.line(screen, CROSS_COLOR, start_desc, end_desc, CROSS_WIDTH)
            start_asc = (col * SQSIZE + OFFSET, row * SQSIZE + SQSIZE - OFFSET)
            end_asc = (col * SQSIZE + SQSIZE - OFFSET, row * SQSIZE + OFFSET)
            pygame.draw.line(screen, CROSS_COLOR, start_asc, end_asc, CROSS_WIDTH)
        elif self.player == 2:
            center = (col * SQSIZE + SQSIZE // 2, row * SQSIZE + SQSIZE // 2)
            pygame.draw.circle(screen, CIRC_COLOR, center, RADIUS, CIRC_WIDTH)

    def next_turn(self):
        # Switches to the next player
        self.player = self.player % 2 + 1

    def isover(self):
        # Checks if the game is over
        result = self.board.final_state(show=True)
        if result != 0:
            self.winner = result
            return True
        elif self.board.isfull():
            self.winner = None
            return True
        return False

    def reset(self):
        # Clear all game elements and reset the board
        self.board = Board()  # Reset to a new board instance
        self.player = 1  # Start with player 1
        self.running = True  # Set game to active
        self.winner = None  # Clear the winner
        screen.fill(BG_COLOR)  # Clear the screen to background color
        self.show_lines()  # Redraw the empty board lines
        pygame.display.update()  # Ensure the display is updated immediately


def main():
    game_mode = menu_screen()  # Initial menu screen to select mode
    game = Game()  # Initialize the game
    game.gamemode = game_mode  # Set game mode based on selection
    board = game.board
    ai = game.ai
    game_over = False  # Track if game is over

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_g:
                    game.change_gamemode()
                if event.key == pygame.K_r:
                    game.reset()
                    board = game.board
                    ai = game.ai
                    game.gamemode = game_mode

                if event.key == pygame.K_0:
                    ai.level = 0
                if event.key == pygame.K_1:
                    ai.level = 1

            if event.type == pygame.MOUSEBUTTONDOWN and game.running:
                pos = event.pos
                row = pos[1] // SQSIZE
                col = pos[0] // SQSIZE
                if board.empty_sqr(row, col):
                    game.make_move(row, col)
                    if game.isover():
                        game.running = False
                        game_over = True  # Set game over to True

        # AI's turn if game is in 'ai' mode
        if game.gamemode == 'ai' and game.player == ai.player and game.running:
            pygame.display.update()
            row, col = ai.eval(board)
            game.make_move(row, col)
            if game.isover():
                game.running = False
                game_over = True

        # Display winner, wait for 3 seconds, and then transition to post-game screen if game is over
        if game_over:
            game.display_winner()  # Display the winner message and strike-through
            pygame.display.update()  # Update the display to show the strike-through
            pygame.time.delay(3000)  # Wait for 3 seconds

            # Show the post-game screen after the delay
            choice = post_game_screen()

            # Handle the player's choice
            if choice == "new_game":
                game.reset()  # Reset the game if "New Game" is selected
                game_over = False  # Reset game over status
            elif choice == "exit":
                pygame.quit()
                sys.exit()

        pygame.display.update()  # Update the display each frame

main()
