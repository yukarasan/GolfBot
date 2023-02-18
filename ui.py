import tkinter as tk

class GolfBot:
    def __init__(self):
        self.count = 0
        self.window = tk.Tk()
        self.window.title("GolfBot")
        self.window.geometry("300x200")
        self.window.configure(background="#F5F5F5")

        self.title_label = tk.Label(self.window, text="GolfBot", font=("Helvetica", 24, "bold"), fg="#1F618D", bg="#F5F5F5")
        self.title_label.pack(pady=10)

        self.count_label = tk.Label(self.window, text=f"Samlede bolde: {self.count}", font=("Helvetica", 16), fg="#1F618D", bg="#F5F5F5")
        self.count_label.pack(pady=10)

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

bot = GolfBot()
bot.window.mainloop()