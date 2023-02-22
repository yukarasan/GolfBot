import tkinter as tk

class GolfBot:
    def __init__(self):
        self.count = 0
        self.status = "Ready..."
        self.window = tk.Tk()
        self.window.title("GolfBot")
        self.window.geometry("600x400")
        self.window.configure(background="#F5F5F5")

        self.title_label = tk.Label(self.window, text="GolfBot", font=("Helvetica", 24, "bold"), fg="#1F618D", bg="#F5F5F5")
        self.title_label.pack(pady=10)

        self.status_label = tk.Label(self.window, text=f"Status: {self.status}", font=("Helvetica", 16), fg="#1F618D", bg="#F5F5F5")
        self.status_label.pack(pady=10)

        self.count_label = tk.Label(self.window, text=f"Samlede bolde: {self.count}", font=("Helvetica", 16), fg="#1F618D", bg="#F5F5F5")
        self.count_label.pack(pady=10)

        self.init_button = tk.Button(self.window, text="Start", font=("Helvetica", 14), fg="#897287", bg="#1F618D", width=10, height=2, command=self.initializeBot)
        self.init_button.pack(padx=10)

        self.plus_button = tk.Button(self.window, text="+", font=("Helvetica", 14), fg="#897287", bg="#1F618D", width=2, height=2, command=self.increment_count)
        self.plus_button.pack(side=tk.LEFT, padx=10)

        self.minus_button = tk.Button(self.window, text="-", font=("Helvetica", 14), fg="#897287", bg="#1F618D", width=2, height=2, command=self.decrement_count)
        self.minus_button.pack(side=tk.RIGHT, padx=10)
        self.window.update_idletasks()

    def increment_count(self):
        self.count += 1
        self.count_label.config(text=f"Samlede bolde: {self.count}")

    def decrement_count(self):
        if self.count > 0:
            self.count -= 1
            self.count_label.config(text=f"Samlede bolde: {self.count}")

    def initializeBot(self):
        self.status = "initializing..."
        self.status_label.config(text=f"Status: {self.status}")

bot = GolfBot()
bot.window.mainloop()