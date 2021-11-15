python_tests = [
  'test_metadata.py',
]

foreach t : python_tests
  test(t, python3,
    args: [files(t)],
    env: test_envs,
    suite: 'python',
  )
endforeach