all:
	@make -C balancebot --no-print-directory
	@make -C test_motors --no-print-directory

clean:
	@make -C balancebot -s clean
	@make -C test_motors -s clean
