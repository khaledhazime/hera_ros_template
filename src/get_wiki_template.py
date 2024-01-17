import subprocess
import sys
import os

class WikiSetup:
    """Handles setting up a GitHub wiki from a template."""

    def __init__(self, template_user_or_org, template_repo_name, new_user_or_org, new_repo_name, gh_username, gh_token):
        """Initializes the WikiSetup with repository details."""
        self.template_wiki_url = f"https://{gh_username}:{gh_token}@github.com/{template_user_or_org}/{template_repo_name}.wiki.git"
        self.new_repo_wiki_url = f"https://github.com/{new_user_or_org}/{new_repo_name}.wiki.git"
        self.gh_username = gh_username
        self.gh_token = gh_token

    def run_command(self, command):
        """Executes a given shell command."""
        try:
            subprocess.check_call(command, shell=True)
        except subprocess.CalledProcessError as e:
            print(f"Error executing command: {e}", file=sys.stderr)
            sys.exit(1)

    def setup(self):
        """Clones the template wiki, sets up the new remote, and pushes to the new repo."""
        print("Cloning template wiki...")
        self.run_command(f"git clone {self.template_wiki_url} template-wiki")

        # Change directory and setup new remote
        try:
            original_dir = os.getcwd()
            os.chdir(os.path.join(original_dir, "template-wiki"))
            print("Setting up new remote...")
            self.run_command("git remote remove origin")
            self.run_command(f"git remote add origin https://{self.gh_username}:{self.gh_token}@{self.new_repo_wiki_url}")
            print("Pushing to new repository's wiki...")
            self.run_command("git push origin master")
        finally:
            os.chdir(original_dir)
            self.run_command("rm -rf template-wiki")

        print("Wiki setup complete.")

if __name__ == "__main__":
    template_user_or_org = 'robofeiathome'
    template_repo_name = 'hera_ros_template'
    new_user_or_org = input("Enter the username/organization for the new repository: ")
    new_repo_name = input("Enter the name of the new repository: ")
    gh_username = input("Enter your GitHub username: ")
    gh_token = input("Enter your GitHub token: ")

    wiki_setup = WikiSetup(template_user_or_org, template_repo_name, new_user_or_org, new_repo_name, gh_username, gh_token)
    wiki_setup.setup()
