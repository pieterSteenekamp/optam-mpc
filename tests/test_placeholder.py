def test_project_structure():
    """Verify the project structure is set up correctly."""
    import optam_mpc
    assert hasattr(optam_mpc, "__version__")
    print(f"OptAM-MPC version {optam_mpc.__version__} is ready")
