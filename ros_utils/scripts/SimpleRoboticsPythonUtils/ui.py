#! /usr/bin/python3
def ask_yes_no(question: str) -> bool:
    """Ask user a binary question

    Args:
        question (str): Question to ask

    Returns:
        bool: user's answer to the question
    """
    res = input(question + "\n" + "[Yy/Nn/Qq]" )
    res = res.lower()
    if res == "q":
        print("Program aborted")
        exit(1)
    elif res == "y":
        return True
    elif res == "n":
        return False