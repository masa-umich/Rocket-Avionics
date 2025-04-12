### I'm bad at compiling C, so just use this for now:

```sh
cd board-drivers/TCP/tests/
```

- Message Tests:

```sh
gcc -I ../server/inc -o messages_tests messages_tests.c ../server/src/messages.c
./messages_tests
rm messages_tests
```
