'''End-to-end tests for PbD.'''

import interaction


class EndToEndTests:
    def main(self):
        '''Runs tests.'''
        # Create object
        self.interaction = interaction.Interaction()

        # Run tests.
        self.test_startup()

    def test_startup(self):
        pass

if __name__ == '__main__':
    EndToEndTests().main()
